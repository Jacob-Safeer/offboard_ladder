#!/usr/bin/env python3
"""
FrontApproach with proper PX4 Offboard activation:
 - Pilot arms + takes off manually.
 - Pilot flips to OFFBOARD in the air.
 - Node begins approach behavior only after OFFBOARD entry.
 - Publishes heartbeat at 50 Hz.
 - Uses velocity control properly for PX4.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleStatus,
    OffboardControlMode,
)


def quat_normalize(q):
    norm = np.linalg.norm(q)
    return q if norm == 0.0 else q / norm


def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quat_to_rot(q):
    w, x, y, z = q
    return np.array(
        [
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ],
        dtype=float,
    )


def rot_to_quat(R):
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    tr = m00 + m11 + m22

    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (m21 - m12) / S
        y = (m02 - m20) / S
        z = (m10 - m01) / S
    elif m00 > m11 and m00 > m22:
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2
        w = (m21 - m12) / S
        x = 0.25 * S
        y = (m01 + m10) / S
        z = (m02 + m20) / S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2
        w = (m02 - m20) / S
        x = (m01 + m10) / S
        y = 0.25 * S
        z = (m12 + m21) / S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2
        w = (m10 - m01) / S
        x = (m02 + m20) / S
        y = (m12 + m21) / S
        z = 0.25 * S

    return quat_normalize(np.array([w, x, y, z]))


class FrontApproachNode(Node):

    class State:
        WAIT_OFFBOARD = "WAIT_OFFBOARD"
        SEARCH = "SEARCH"
        APPROACH = "APPROACH"
        FINISHED = "FINISHED"

    def __init__(self):
        super().__init__("front_approach_py")

        # ------------------------------------------------------------------
        # QoS
        # ------------------------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # ------------------------------------------------------------------
        # PX4 Subscribers
        # ------------------------------------------------------------------
        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.status_cb,
            qos
        )
        self.status_sub_v1 = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self.status_cb,
            qos
        )

        self.front_target_sub = self.create_subscription(
            PoseStamped,
            "/front/target_pose",
            self.front_target_cb,
            qos_profile_sensor_data
        )

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.local_position_cb,
            qos_profile_sensor_data
        )

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.attitude_cb,
            qos
        )

        # ------------------------------------------------------------------
        # PX4 Publishers
        # ------------------------------------------------------------------
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            qos
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            qos
        )

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.param_hold_distance = 1.0
        self.param_target_timeout = 3.0
        self.param_kp_xy = 0.6
        self.param_ki_xy = 0.0
        self.param_kd_xy = 0.0
        self.param_max_velocity_xy = 1.0
        self.param_integral_limit = 1.5
        self.param_kp_z = 0.5
        self.param_max_velocity_z = 1.0

        # ------------------------------------------------------------------
        # Internal State
        # ------------------------------------------------------------------
        self.state = self.State.WAIT_OFFBOARD
        self.offboard_active = False

        self.front_tag = None
        self.last_tag_time = None

        self.local_position = None
        self.attitude_quat = None
        self.prev_status = None

        self.integral_xy = np.zeros(2)
        self.prev_error_xy = np.zeros(2)
        self.has_prev_error = False

        self.front_optical_to_body = rot_to_quat(
            np.array([[0,0,1],[1,0,0],[0,1,0]], dtype=float)
        )

        self.prev_time = self.get_clock().now()

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        self.timer = self.create_timer(1/50.0, self.control_loop)
        self.heartbeat_timer = self.create_timer(0.01, self.publish_heartbeat)


    # ==================================================================
    # PX4 MODE ACTIVATION
    # ==================================================================
    def status_cb(self, msg):
        if self.prev_status is None:
            self.prev_status = msg.nav_state
            return

        # OFFBOARD rising edge
        if (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
            self.prev_status != VehicleStatus.NAVIGATION_STATE_OFFBOARD):

            self.get_logger().info("OFFBOARD ACTIVATED — FrontApproach starting.")
            self.offboard_active = True
            self.state = self.State.SEARCH

        self.prev_status = msg.nav_state


    def publish_heartbeat(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        msg.velocity = True        # we are using velocity offboard
        msg.position = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_mode_pub.publish(msg)


    # ==================================================================
    # TAG CALLBACKS / TRANSFORM
    # ==================================================================
    def front_target_cb(self, msg):
        pos = np.array([msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z], dtype=float)

        if not np.isfinite(pos).all():
            return

        if self.local_position is None or self.attitude_quat is None:
            return

        R_vehicle = quat_to_rot(self.attitude_quat)
        R_cam = quat_to_rot(self.front_optical_to_body)

        tag_body = R_cam @ pos
        world = np.array([self.local_position.x,
                          self.local_position.y,
                          self.local_position.z]) + (R_vehicle @ tag_body)

        self.front_tag = world
        self.last_tag_time = self.get_clock().now()


    def target_expired(self, now):
        if self.front_tag is None or self.last_tag_time is None:
            return True
        return (now.nanoseconds - self.last_tag_time.nanoseconds)*1e-9 > self.param_target_timeout


    # ==================================================================
    # POSITION / ATTITUDE
    # ==================================================================
    def local_position_cb(self, msg):
        self.local_position = msg

    def attitude_cb(self, msg):
        self.attitude_quat = quat_normalize(np.array(msg.q, dtype=float))


    # ==================================================================
    # CONTROL LOOP
    # ==================================================================
    def control_loop(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.prev_time.nanoseconds)*1e-9
        self.prev_time = now
        if dt <= 0.0:
            return

        if not self.offboard_active:
            return

        if self.local_position is None or self.attitude_quat is None:
            return

        # ----------------------------------------------------------------------
        # WAIT_OFFBOARD STATE
        # ----------------------------------------------------------------------
        if self.state == self.State.WAIT_OFFBOARD:
            return

        # ----------------------------------------------------------------------
        # SEARCH: hold still until tag appears
        # ----------------------------------------------------------------------
        if self.state == self.State.SEARCH:
            if not self.target_expired(now):
                self.get_logger().info("Front tag acquired → APPROACH")
                self.state = self.State.APPROACH

            self.publish_position_hold()
            return

        # ----------------------------------------------------------------------
        # APPROACH: run your original PID-vector approach logic
        # ----------------------------------------------------------------------
        if self.state == self.State.APPROACH:
            if self.target_expired(now):
                self.get_logger().info("Tag lost → SEARCH")
                self.state = self.State.SEARCH
                return

            vehicle = np.array([self.local_position.x,
                                self.local_position.y,
                                self.local_position.z])

            tag = self.front_tag
            to_tag = tag - vehicle

            delta_xy = to_tag[:2]
            dist_xy = np.linalg.norm(delta_xy)

            hold = self.param_hold_distance
            if dist_xy > hold:
                desired_xy = delta_xy - delta_xy/dist_xy * hold
            else:
                desired_xy = np.zeros(2)

            target_pos = np.array([
                vehicle[0] + desired_xy[0],
                vehicle[1] + desired_xy[1],
                tag[2]
            ])

            # --- PID XY ---
            error_xy = target_pos[:2] - vehicle[:2]
            self.integral_xy += error_xy * dt
            self.integral_xy = np.clip(self.integral_xy,
                                       -self.param_integral_limit,
                                       self.param_integral_limit)

            if self.has_prev_error:
                derivative_xy = (error_xy - self.prev_error_xy)/dt
            else:
                derivative_xy = np.zeros(2)
                self.has_prev_error = True

            vel_xy = (self.param_kp_xy * error_xy +
                      self.param_ki_xy * self.integral_xy +
                      self.param_kd_xy * derivative_xy)

            speed = np.linalg.norm(vel_xy)
            if speed > self.param_max_velocity_xy:
                vel_xy = vel_xy/speed * self.param_max_velocity_xy

            # --- PID Z ---
            error_z = target_pos[2] - vehicle[2]
            vel_z = float(np.clip(self.param_kp_z * error_z,
                                  -self.param_max_velocity_z,
                                  self.param_max_velocity_z))

            velocity_cmd = np.array([vel_xy[0], vel_xy[1], vel_z], dtype=float)

            yaw = math.atan2(to_tag[1], to_tag[0])
            self.publish_velocity_cmd(velocity_cmd, yaw)

            self.prev_error_xy = error_xy

            # Check stopping condition
            if self.position_reached(target_pos):
                self.get_logger().info("Final offset reached → FINISHED")
                self.state = self.State.FINISHED
            return

        # ----------------------------------------------------------------------
        # FINISHED: hold
        # ----------------------------------------------------------------------
        if self.state == self.State.FINISHED:
            self.publish_position_hold()
            return


    # ==================================================================
    # HELPERS
    # ==================================================================
    def publish_position_hold(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds//1000

        msg.position = [float(self.local_position.x),
                        float(self.local_position.y),
                        float(self.local_position.z)]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan]*3
        msg.jerk = [math.nan]*3
        msg.yaw = math.nan
        self.setpoint_pub.publish(msg)


    def publish_velocity_cmd(self, vel, yaw):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds//1000
        msg.velocity = vel.astype(np.float32).tolist()
        msg.position = [math.nan]*3
        msg.acceleration = [math.nan]*3
        msg.jerk = [math.nan]*3
        msg.yaw = yaw
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)


    def position_reached(self, target):
        pos = np.array([self.local_position.x,
                        self.local_position.y,
                        self.local_position.z])
        vel = np.array([self.local_position.vx,
                        self.local_position.vy,
                        self.local_position.vz])

        return (
            np.linalg.norm(target[:2] - pos[:2]) < 0.25 and
            abs(target[2] - pos[2]) < 0.25 and
            np.linalg.norm(vel) < 0.25
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontApproachNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
