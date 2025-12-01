#!/usr/bin/env python3

import rclpy
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleAttitude,
)
from geometry_msgs.msg import PoseStamped


class OffboardShipLandNode(Node):
    """Offboard controller for ship landing. 
       Activates ONLY when pilot manually switches to OFFBOARD in the air."""

    def __init__(self) -> None:
        super().__init__('offboard_ship_land_node')
        self.get_logger().info("Offboard Ship Land Node Alive!")

        # QoS for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_status_subscriber_v1 = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile_sensor_data)

        self.tag_subscriber = self.create_subscription(
            PoseStamped,
            '/front/target_pose',
            self.tag_pose_callback,
            qos_profile_sensor_data
        )

        # NEW: attitude subscriber for yaw
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile
        )

        # ---------------------------------------------------------
        # Internal state
        # ---------------------------------------------------------
        self.vehicle_local_position = None   # (x, y, z)
        self.tag_pose = None                 # (x, y, z) from vision
        self.yaw = None                      # from quaternion
        self.altitude = None                 # locked at activation

        self.rate = 20                       # control loop Hz
        self.mode_active = False             # becomes True on OFFBOARD entry
        self.last_nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.tag_align_timer = None
        self.land_start_time = None
        self.hover_cords = None

        # Timer: heartbeat only
        self.timer = self.create_timer(1.0 / self.rate, self.heartbeat_timer)

    # =============================================================
    # HEARTBEAT: keeps offboard alive if pilot switches to it
    # =============================================================
    def heartbeat_timer(self):
        self.publish_offboard_control_heartbeat_signal_position()

    # =============================================================
    # CALLBACKS
    # =============================================================
    def vehicle_status_callback(self, vehicle_status):
        """Detect OFFBOARD activation by pilot."""
        prev = self.last_nav_state
        self.last_nav_state = vehicle_status.nav_state

        # Rising edge: OFFBOARD just activated
        if (vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
                prev != VehicleStatus.NAVIGATION_STATE_OFFBOARD):

            self.get_logger().info("OFFBOARD mode ACTIVATED by pilot!")

            # Try activating behavior
            self.mode_active = True
            self.on_activation()

        self.vehicle_status = vehicle_status

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        # PX4 NED coordinates: x=North, y=East, z=Down
        self.vehicle_local_position = (msg.x, msg.y, msg.z)

    def vehicle_attitude_callback(self, msg: VehicleAttitude):
        """Extract yaw from quaternion."""
        q = msg.q  # [w, x, y, z]
        w, x, y, z = q[0], q[1], q[2], q[3]

        # yaw extraction
        self.yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )

    def tag_pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.tag_pose = (x, y, z)

    # =============================================================
    # ACTIVATION (when pilot enters OFFBOARD)
    # =============================================================
    def on_activation(self):
        """Called once the pilot switches to OFFBOARD in the air."""
        if self.vehicle_local_position is None:
            self.get_logger().warn("No local position yet — waiting…", throttle_duration_sec=2.0)
            return

        if self.yaw is None:
            self.get_logger().warn("No yaw yet — waiting…", throttle_duration_sec=2.0)
            return

        if self.tag_pose is None:
            self.get_logger().warn("No tag detected — waiting…", throttle_duration_sec=2.0)
            return

        # Lock altitude at activation time
        _, _, curr_z = self.vehicle_local_position
        self.altitude = curr_z

        # Start tag-alignment behavior
        self.tag_align_timer = self.create_timer(1.0 / self.rate, self.offboard_move_callback)
        self.get_logger().info("Tag alignment behavior STARTED.")

    # =============================================================
    # MAIN TAG ALIGNMENT / LANDING LOGIC
    # =============================================================
    def offboard_move_callback(self):
        if not self.mode_active:
            return

        if self.tag_pose is None:
            self.get_logger().warn("Tag lost — hovering.", throttle_duration_sec=2.0)
            return

        if self.vehicle_local_position is None or self.yaw is None:
            return

        x_tag, y_tag, z_tag = self.tag_pose

        # Horizontal alignment: tag x==0 means centered in camera frame
        horizontal_align = (-0.3 < x_tag < 0.3)
        ready_to_land = horizontal_align and z_tag < 2.0

        if self.land_start_time is None:

            if ready_to_land:
                # Hover exactly above target
                self.hover_cords = self.vehicle_local_position
                self.land_start_time = time.time()
                self.altitude = self.hover_cords[2]
                print("Ready to land — entering hover phase.")

            elif horizontal_align:
                print("Centered horizontally — moving forward.")
                self.publish_move_forward_setpoint(x_tag)

            elif x_tag >= 0.2:
                print("Tag right — moving right.")
                self.publish_move_right_setpoint()

            elif x_tag <= -0.2:
                print("Tag left — moving left.")
                self.publish_move_left_setpoint()

        else:
            # Hovering above target for stabilization
            if time.time() - self.land_start_time > 1.5:
                print("Landing initiated.")
                self.tag_align_timer.cancel()  # Stop control loop
            else:
                print("Hovering before descent.")
                self.publish_current_hover_setpoint(
                    self.hover_cords[0], self.hover_cords[1]
                )

    # =============================================================
    # FRAME-SAFE MOVEMENT FUNCTIONS
    # =============================================================
    def _ensure_pose_and_yaw(self) -> bool:
        if self.vehicle_local_position is None:
            return False
        if self.yaw is None:
            return False
        return True

    def publish_move_left_setpoint(self):
        """Move left in BODY frame (negative Y_body)."""
        if not self._ensure_pose_and_yaw():
            return
        curr_x, curr_y, curr_z = self.vehicle_local_position
        step = 4.0 / self.rate

        dx_b = 0.0
        dy_b = -step

        # body → NED
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        x_off = dx_b * cos_yaw - dy_b * sin_yaw
        y_off = dx_b * sin_yaw + dy_b * cos_yaw

        self._publish_position(curr_x + x_off, curr_y + y_off, self.altitude)

    def publish_move_right_setpoint(self):
        """Move right in BODY frame (positive Y_body)."""
        if not self._ensure_pose_and_yaw():
            return
        curr_x, curr_y, curr_z = self.vehicle_local_position
        step = 4.0 / self.rate

        dx_b = 0.0
        dy_b = step

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        x_off = dx_b * cos_yaw - dy_b * sin_yaw
        y_off = dx_b * sin_yaw + dy_b * cos_yaw

        self._publish_position(curr_x + x_off, curr_y + y_off, self.altitude)

    def publish_move_forward_setpoint(self, tag_displacement):
        """Move forward in BODY frame, with lateral adjustment."""
        if not self._ensure_pose_and_yaw():
            return
        curr_x, curr_y, curr_z = self.vehicle_local_position

        forward_step = 6.0 / self.rate

        if tag_displacement > 0:
            side_adj = min(5.0 * tag_displacement, 2.0)
        else:
            side_adj = max(5.0 * tag_displacement, -2.0)

        dx_b = forward_step
        dy_b = side_adj / self.rate

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        x_off = dx_b * cos_yaw - dy_b * sin_yaw
        y_off = dx_b * sin_yaw + dy_b * cos_yaw

        self._publish_position(curr_x + x_off, curr_y + y_off, self.altitude)

    def publish_current_hover_setpoint(self, x, y):
        self._publish_position(x, y, self.altitude)

    # =============================================================
    # GENERIC POSITION SETPOINT
    # =============================================================
    def _publish_position(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float('nan')  # don't force yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    # =============================================================
    # OFFBOARD HEARTBEAT
    # =============================================================
    def publish_offboard_control_heartbeat_signal_position(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    # =============================================================
    # VEHICLE COMMAND WRAPPER (still needed for landing)
    # =============================================================
    def publish_vehicle_command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardShipLandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
