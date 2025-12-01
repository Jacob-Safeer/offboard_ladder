#!/usr/bin/env python3

############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

import math
from typing import Optional

import numpy as np
import rclpy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)


class OffboardTest(Node):

    def __init__(self) -> None:
        super().__init__('offboard_test')

        self.get_logger().info("Offboard Test Node Alive!")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

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

        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            'fmu/in/offboard_control_mode',
            qos_profile)
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint,
            'fmu/in/trajectory_setpoint',
            qos_profile)

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        self.declare_parameter('lateral_amplitude', 1.0)  # meters
        self.declare_parameter('lateral_rate', 0.3)  # rad/s for slow oscillation
        self.declare_parameter('lock_to_current_position', True)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.theta = 0.0
        self.amplitude = float(self.get_parameter('lateral_amplitude').value)
        self.angular_rate = float(self.get_parameter('lateral_rate').value)
        self.lock_center = bool(self.get_parameter('lock_to_current_position').value)

        self.have_local_position = False
        self.local_position = np.zeros(3)
        self.start_position: Optional[np.ndarray] = None if self.lock_center else np.zeros(3)

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        # TODO: handle NED->ENU transformation if needed
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.local_position = np.array([msg.x, msg.y, msg.z], dtype=float)
        self.have_local_position = True

    def lock_start_position(self) -> None:
        if self.start_position is not None:
            return

        if not self.lock_center:
            self.start_position = np.zeros(3)
            return

        if not self.have_local_position:
            self.get_logger().warn('Waiting for local position to lock start position...',
                                   throttle_duration_sec=2.0)
            return

        self.start_position = self.local_position.copy()
        self.get_logger().info(
            f"Start position locked at ({self.start_position[0]:.1f}, "
            f"{self.start_position[1]:.1f}, {self.start_position[2]:.1f})")

    def cmdloop_callback(self) -> None:
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        if (self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD or
                self.arming_state != VehicleStatus.ARMING_STATE_ARMED):
            return

        self.lock_start_position()
        if self.start_position is None:
            return

        # Slowly oscillate 1 m left/right relative to the start position (Y axis).
        lateral_offset = self.amplitude * math.sin(self.theta)
        x = self.start_position[0]
        y = self.start_position[1] + lateral_offset
        z = self.start_position[2]

        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = offboard_msg.timestamp
        trajectory_msg.position[0] = float(x)
        trajectory_msg.position[1] = float(y)
        trajectory_msg.position[2] = float(z)
        trajectory_msg.velocity[0] = float('nan')
        trajectory_msg.velocity[1] = float('nan')
        trajectory_msg.velocity[2] = float('nan')
        trajectory_msg.yaw = float('nan')
        trajectory_msg.yawspeed = float('nan')
        self.publisher_trajectory.publish(trajectory_msg)

        self.theta += self.angular_rate * self.dt


def main(args=None) -> None:
    rclpy.init(args=args)

    offboard_test = OffboardTest()

    rclpy.spin(offboard_test)

    offboard_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
