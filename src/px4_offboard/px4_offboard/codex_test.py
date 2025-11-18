#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleLocalPosition,
    VehicleStatus,
)


class CodexTestNode(Node):
    """
    Simplest possible tag-aware offboard node:

    - Publishes the offboard heartbeat.
    - When the vehicle is ARMED+OFFBOARD it re-publishes the current position
      setpoint so the drone holds its place.
    - Subscribes to /front/target_pose and logs tag detections so we can verify
      the pipeline end-to-end before adding closed-loop behavior.
    """

    def __init__(self) -> None:
        super().__init__("codex_test")

        qos_best_effort = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_best_effort
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_best_effort
        )

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_best_effort,
        )

        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.vehicle_local_position_callback,
            qos_profile_sensor_data,
        )

        self.tag_sub = self.create_subscription(
            PoseStamped,
            "/front/target_pose",
            self.tag_callback,
            qos_profile_sensor_data,
        )

        self.vehicle_status = VehicleStatus()
        self.vehicle_position = None  # type: ignore[assignment]
        self.tag_pose = None
        self.tag_seq = 0

        self.desired_distance = 3.0  # meters
        self.k_forward = 0.4
        self.max_step = 0.5

        self.logged_waiting_for_pose = False
        self.logged_waiting_for_tag = False

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    # ------------------------------------------------------------------ callbacks

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        pos = [msg.x, msg.y, msg.z]
        if all(math.isfinite(val) for val in pos):
            self.vehicle_position = pos

    def tag_callback(self, msg: PoseStamped) -> None:
        self.tag_pose = msg.pose
        self.tag_seq += 1
        self.logged_waiting_for_tag = False
        self.get_logger().info(
            f"Tag #{self.tag_seq} detected: "
            f"x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f} z={msg.pose.position.z:.2f}"
        )

    # ------------------------------------------------------------------ helpers

    def control_loop(self) -> None:
        self.publish_offboard_heartbeat()

        if not self.is_offboard_active():
            return

        if self.vehicle_position is None:
            if not self.logged_waiting_for_pose:
                self.get_logger().warn(
                    "Still waiting for /fmu/out/vehicle_local_position"
                )
                self.logged_waiting_for_pose = True
            return
        self.logged_waiting_for_pose = False

        if self.tag_pose is None:
            if not self.logged_waiting_for_tag:
                self.get_logger().info("Waiting for /front/target_pose messages...")
                self.logged_waiting_for_tag = True
            self.publish_position_setpoint(self.vehicle_position)
            return

        # Adjust only along the vehicle X axis based on tag range
        tag_z = self.tag_pose.position.z
        if not math.isfinite(tag_z):
            self.get_logger().warn("Tag range invalid; holding position")
            self.publish_position_setpoint(self.vehicle_position)
            return

        forward_error = tag_z - self.desired_distance
        # If tag_z increases, we're farther away. Apply negative gain so we fly forward.
        delta = self._clamp(-self.k_forward * forward_error, -self.max_step, self.max_step)

        desired_position = self.vehicle_position.copy()
        desired_position[0] += delta  # assume PX4 local X points forward

        direction = "forward" if delta > 0 else ("backward" if delta < 0 else "hold")
        self.get_logger().info(
            f"Tag distance={tag_z:.2f} m → {direction} step {delta:.2f} m (target {self.desired_distance:.1f} m)"
        )

        self.publish_position_setpoint(desired_position)

    def is_offboard_active(self) -> bool:
        return (
            self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def publish_offboard_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_position_setpoint(self, position_ned) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float(position_ned[0]), float(position_ned[1]), float(position_ned[2])]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = 0.0
        msg.yawspeed = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CodexTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
