import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class ManualControl(Node):
    """Publishes arm state and velocity commands for the velocity controller."""

    def __init__(self) -> None:
        super().__init__('px4_offboard_control')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher_velocity = self.create_publisher(
            Twist,
            '/offboard_velocity_cmd',
            qos
        )
        self.publisher_arm = self.create_publisher(
            Bool,
            '/arm_message',
            qos
        )

        self.declare_parameter('arm', False)
        self.declare_parameter('vx', 0.0)
        self.declare_parameter('vy', 0.0)
        self.declare_parameter('vz', 0.0)
        self.declare_parameter('yaw_rate', 0.0)

        self._arm = self.get_parameter('arm').value
        self._velocities = {
            'x': float(self.get_parameter('vx').value),
            'y': float(self.get_parameter('vy').value),
            'z': float(self.get_parameter('vz').value),
            'yaw': float(self.get_parameter('yaw_rate').value),
        }

        self.add_on_set_parameters_callback(self._parameter_callback)

        self._cmd_timer = self.create_timer(0.1, self._publish_commands)

    def _parameter_callback(self, params):
        for param in params:
            if param.name == 'arm':
                self._arm = bool(param.value)
            elif param.name == 'vx':
                self._velocities['x'] = float(param.value)
            elif param.name == 'vy':
                self._velocities['y'] = float(param.value)
            elif param.name == 'vz':
                self._velocities['z'] = float(param.value)
            elif param.name == 'yaw_rate':
                self._velocities['yaw'] = float(param.value)
        return SetParametersResult(successful=True)

    def _publish_commands(self) -> None:
        vel = Twist()
        vel.linear.x = self._velocities['x']
        vel.linear.y = self._velocities['y']
        vel.linear.z = self._velocities['z']
        vel.angular.z = self._velocities['yaw']
        self.publisher_velocity.publish(vel)

        arm_msg = Bool()
        arm_msg.data = self._arm
        self.publisher_arm.publish(arm_msg)


def main(args=None):
    rclpy.init(args=args)
    control = ManualControl()
    try:
        rclpy.spin(control)
    finally:
        control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
