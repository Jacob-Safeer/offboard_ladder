#!/usr/bin/env python3

import math
from typing import Optional

import numpy as np
import rclpy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleAttitude
)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class OffboardTest(Node):

    def __init__(self) -> None:
        super().__init__('offboard_test')

        self.get_logger().info("Offboard Test Node Alive!")

        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)

        self.status_sub_v1 = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile)

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)

        # NEW: attitude subscription (for yaw)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            'fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile)

        # Publishers
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            'fmu/in/offboard_control_mode',
            qos_profile)

        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint,
            'fmu/in/trajectory_setpoint',
            qos_profile)

        # Timer loop
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        # Parameters
        self.declare_parameter('lateral_amplitude', 1.0)
        self.declare_parameter('lateral_rate', 0.3)
        self.declare_parameter('lock_to_current_position', True)

        self.amplitude = float(self.get_parameter('lateral_amplitude').value)
        self.angular_rate = float(self.get_parameter('lateral_rate').value)
        self.lock_center = bool(self.get_parameter('lock_to_current_position').value)

        # Internal state
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.theta = 0.0

        self.have_local_position = False
        self.local_position = np.zeros(3)

        # NEW: store yaw from attitude
        self.yaw = None

        # Start position (motion center)
        self.start_position: Optional[np.ndarray] = None if self.lock_center else np.zeros(3)

    # ---------------------------------------------------------------------
    # CALLBACKS
    # ---------------------------------------------------------------------

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.local_position = np.array([msg.x, msg.y, msg.z], dtype=float)
        self.have_local_position = True

    def vehicle_attitude_callback(self, msg: VehicleAttitude):
        # Quaternion → Yaw (ENU convention)
        q = msg.q  # array of [w, x, y, z]
        w, x, y, z = q[0], q[1], q[2], q[3]

        # yaw = atan2(2(wx + yz), w² + x² - y² - z²)
        self.yaw = math.atan2(2.0 * (w*z + x*y),
                              1.0 - 2.0 * (y*y + z*z))

    # ---------------------------------------------------------------------
    # START POSITION LOCK
    # ---------------------------------------------------------------------

    def lock_start_position(self) -> None:
        if self.start_position is not None:
            return

        if not self.lock_center:
            self.start_position = np.zeros(3)
            return

        if not self.have_local_position:
            self.get_logger().warn(
                'Waiting for local position to lock start position...',
                throttle_duration_sec=2.0)
            return

        self.start_position = self.local_position.copy()
        self.get_logger().info(
            f"Start position locked at ({self.start_position[0]:.1f}, "
            f"{self.start_position[1]:.1f}, {self.start_position[2]:.1f})"
        )

    # ---------------------------------------------------------------------
    # MAIN LOOP
    # ---------------------------------------------------------------------

    def cmdloop_callback(self) -> None:

        # Always publish offboard control mode
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        # Only run offboard logic when armed + in offboard
        if (self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD or
                self.arming_state != VehicleStatus.ARMING_STATE_ARMED):
            return

        self.lock_start_position()
        if self.start_position is None:
            return

        # Need yaw to compute body-relative movement
        if self.yaw is None:
            self.get_logger().warn("Waiting for yaw...", throttle_duration_sec=2.0)
            return

        # -----------------------------------------------------------------
        # BODY-FRAME SIDE-TO-SIDE MOTION
        # -----------------------------------------------------------------

        # Lateral motion in BODY coordinate system
        lateral_offset_body = self.amplitude * math.sin(self.theta)

        # Convert body → world for a pure lateral motion
        x_offset = -lateral_offset_body * math.sin(self.yaw)
        y_offset =  lateral_offset_body * math.cos(self.yaw)

        # Final world position setpoint
        x = self.start_position[0] + x_offset
        y = self.start_position[1] + y_offset
        z = self.start_position[2]

        # Publish setpoint
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = offboard_msg.timestamp
        trajectory_msg.position[0] = float(x)
        trajectory_msg.position[1] = float(y)
        trajectory_msg.position[2] = float(z)
        trajectory_msg.velocity[:] = [float('nan')] * 3
        trajectory_msg.yaw = float('nan')
        trajectory_msg.yawspeed = float('nan')
        self.publisher_trajectory.publish(trajectory_msg)

        # Update phase
        self.theta += self.angular_rate * self.dt


# ------------------------------------------------------------------------------
# MAIN
# ------------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    offboard_test = OffboardTest()
    rclpy.spin(offboard_test)
    offboard_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
