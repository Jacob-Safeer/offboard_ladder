#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class LateralStepOffboard(Node):
    """
    Simple PX4 offboard controller:
      1. Waits for valid local position (after manual takeoff).
      2. Pilot flips to OFFBOARD; the node captures current (x, y, z, yaw).
      3. Moves 1 m "left" (y + 1).
      4. Moves back to the captured start point.
      5. Holds the start point.

    Run this node AFTER the pilot has taken off and stabilized in a position mode.
    It streams setpoints continuously; the pilot chooses when to enter OFFBOARD.
    """

    def __init__(self):
        super().__init__('lateral_step_offboard')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers to PX4
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos,
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos,
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        # Local position subscriber
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos,
        )

        # State
        self.local_position: Optional[VehicleLocalPosition] = None
        self.nav_state: Optional[int] = None
        self.offboard_active = False

        self.ref_x = 0.0
        self.ref_y = 0.0
        self.ref_z = 0.0
        self.ref_yaw = 0.0

        # Targets (x, y, z)
        self.home_target: Optional[Tuple[float, float, float]] = None
        self.left_target: Optional[Tuple[float, float, float]] = None
        self.return_target: Optional[Tuple[float, float, float]] = None
        self.current_target: Optional[Tuple[float, float, float]] = None

        # Simple state machine
        self.state = 'WAIT_FOR_POSITION'
        # States:
        #   WAIT_FOR_POSITION -> WAIT_FOR_OFFBOARD -> GO_LEFT -> RETURN_HOME -> HOLD

        # Main control timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            "LateralStepOffboard node started. "
            "Take off manually first, then start this node while in the air."
        )

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def local_position_callback(self, msg: VehicleLocalPosition):
        self.local_position = msg

        # Track the most recent valid pose for pre-offboard holding.
        if msg.xy_valid and msg.z_valid:
            if not self.offboard_active:
                self.current_target = (msg.x, msg.y, msg.z)
                self.ref_yaw = self._extract_yaw(msg, default=self.ref_yaw)
                self.state = 'WAIT_FOR_OFFBOARD'

        if self.offboard_active:
            self.update_state_machine(msg)

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state

        offboard_now = self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        if offboard_now and not self.offboard_active:
            self.activate_offboard_sequence()
        elif self.offboard_active and not offboard_now:
            self.offboard_active = False
            self.state = 'WAIT_FOR_OFFBOARD'
            self.get_logger().info("Exited OFFBOARD; holding current position.")

    # -------------------------------------------------------------------------
    # Offboard / command helpers
    # -------------------------------------------------------------------------

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # PX4 expects µs

        # We are using position setpoints only
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # µs

        if self.current_target is None:
            # Before we have a target, hold at origin (pilot should not switch yet).
            x, y, z = 0.0, 0.0, 0.0
        else:
            x, y, z = self.current_target

        # PX4 expects NED, z negative when above home
        msg.position = [float(x), float(y), float(z)]
        # Unused fields set to NaN so PX4 ignores them
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]

        msg.yaw = float(self.ref_yaw)
        msg.yawspeed = 0.0

        self.trajectory_setpoint_pub.publish(msg)

    def send_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ):
        """Send a generic VEHICLE_COMMAND to PX4."""
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # µs

        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)

        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_pub.publish(msg)

    def activate_offboard_sequence(self):
        """Capture reference at OFFBOARD entry and start lateral step."""
        if self.local_position is None or not (self.local_position.xy_valid and self.local_position.z_valid):
            self.get_logger().warn("OFFBOARD requested but local position not valid yet; holding.")
            return

        self.ref_x = self.local_position.x
        self.ref_y = self.local_position.y
        self.ref_z = self.local_position.z
        self.ref_yaw = self._extract_yaw(self.local_position, default=self.ref_yaw)

        self.home_target = (self.ref_x, self.ref_y, self.ref_z)
        self.left_target = (self.ref_x, self.ref_y + 1.0, self.ref_z)
        self.return_target = self.home_target

        self.current_target = self.left_target
        self.state = 'GO_LEFT'
        self.offboard_active = True

        self.get_logger().info(
            f"OFFBOARD engaged at x={self.ref_x:.2f}, y={self.ref_y:.2f}, "
            f"z={self.ref_z:.2f}, yaw={self.ref_yaw:.2f} rad. State -> GO_LEFT."
        )

    # -------------------------------------------------------------------------
    # State machine for left/right motion
    # -------------------------------------------------------------------------

    def update_state_machine(self, pos: VehicleLocalPosition):
        if self.state == 'GO_LEFT' and self.left_target is not None:
            if self.is_close_to_target(self.left_target, pos):
                self.state = 'RETURN_HOME'
                self.current_target = self.return_target
                self.get_logger().info(
                    "Reached left target. State -> RETURN_HOME (commanding -1 m in local Y)."
                )

        elif self.state == 'RETURN_HOME' and self.return_target is not None:
            if self.is_close_to_target(self.return_target, pos):
                self.state = 'HOLD'
                # Keep holding at start point
                self.current_target = self.return_target
                self.get_logger().info("Returned to start point. State -> HOLD.")

        # In HOLD, we just keep sending the same target; pilot can switch modes
        # or land manually.

    @staticmethod
    def is_close_to_target(target: Tuple[float, float, float],
                           pos: VehicleLocalPosition,
                           tol: float = 0.15) -> bool:
        """Check if |pos - target| < tol in XY and Z."""
        dx = pos.x - target[0]
        dy = pos.y - target[1]
        dz = pos.z - target[2]
        dist_xy = math.hypot(dx, dy)
        return dist_xy < tol and abs(dz) < tol

    # -------------------------------------------------------------------------
    # Main control loop
    # -------------------------------------------------------------------------

    def control_loop(self):
        # Keep holding the most recent valid position until OFFBOARD is active.
        if not self.offboard_active and self.local_position and self.local_position.xy_valid and self.local_position.z_valid:
            self.current_target = (self.local_position.x, self.local_position.y, self.local_position.z)

        # Always publish OffboardControlMode and TrajectorySetpoint at high rate
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

    @staticmethod
    def _extract_yaw(pos: VehicleLocalPosition, default: float) -> float:
        """Return yaw if valid; otherwise, keep last yaw."""
        try:
            if hasattr(pos, 'heading_good_for_control') and not pos.heading_good_for_control:
                return default
        except Exception:
            pass
        return float(pos.heading)


def main(args=None):
    rclpy.init(args=args)
    node = LateralStepOffboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down LateralStepOffboard.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
