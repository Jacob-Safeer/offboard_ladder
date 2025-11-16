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
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)


class CircleOffboard(Node):

    def __init__(self) -> None:
        super().__init__('circle_offboard_mode')

        # QoS configuration mirrors the base offboard_control example
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile_sub)
        self.status_sub_v1 = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile_sub)

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile_sub)

        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            'fmu/in/offboard_control_mode',
            qos_profile_pub)
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint,
            'fmu/in/trajectory_setpoint',
            qos_profile_pub)

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        self.declare_parameter('radius', 5.0)
        self.declare_parameter('omega', 0.4)  # rad/s
        self.declare_parameter('altitude', 5.0)
        self.declare_parameter('lock_to_current_position', True)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.theta = 0.0

        self.radius = float(self.get_parameter('radius').value)
        self.omega = float(self.get_parameter('omega').value)
        self.altitude = float(self.get_parameter('altitude').value)
        self.lock_center = bool(self.get_parameter('lock_to_current_position').value)

        self.have_local_position = False
        self.local_position = np.zeros(3)
        self.circle_center: Optional[np.ndarray] = None if self.lock_center else np.zeros(2)

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        # TODO: handle NED->ENU transformation if needed
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.local_position = np.array([msg.x, msg.y, msg.z], dtype=float)
        self.have_local_position = True

    def lock_circle_center(self) -> None:
        if self.circle_center is not None:
            return

        if not self.lock_center:
            self.circle_center = np.zeros(2)
            return

        if not self.have_local_position:
            self.get_logger().warn('Waiting for local position to lock circle center...',
                                   throttle_duration_sec=2.0)
            return

        self.circle_center = self.local_position[:2].copy()
        self.get_logger().info(
            f"Circle center locked at ({self.circle_center[0]:.1f}, {self.circle_center[1]:.1f})")

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

        self.lock_circle_center()
        if self.circle_center is None:
            return

        angle = self.theta
        x = self.circle_center[0] + self.radius * math.cos(angle)
        y = self.circle_center[1] + self.radius * math.sin(angle)
        z = -self.altitude

        vx = -self.radius * self.omega * math.sin(angle)
        vy = self.radius * self.omega * math.cos(angle)
        yaw = math.atan2(vy, vx)

        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = offboard_msg.timestamp
        trajectory_msg.position[0] = float(x)
        trajectory_msg.position[1] = float(y)
        trajectory_msg.position[2] = float(z)
        trajectory_msg.velocity[0] = float('nan')
        trajectory_msg.velocity[1] = float('nan')
        trajectory_msg.velocity[2] = float('nan')
        trajectory_msg.yaw = float(yaw)
        trajectory_msg.yawspeed = float('nan')
        self.publisher_trajectory.publish(trajectory_msg)

        self.theta += self.omega * self.dt


def main(args=None) -> None:
    rclpy.init(args=args)

    circle_offboard = CircleOffboard()

    rclpy.spin(circle_offboard)

    circle_offboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
