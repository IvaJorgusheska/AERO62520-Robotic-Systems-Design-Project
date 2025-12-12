import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class CmdVelTransformer(Node):


    def __init__(self):
        super().__init__('cmd_vel_transformer')


        self.declare_parameter('wheel_base', 0.5)
        self.declare_parameter('track_width', 0.4)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('use_sim_time', False)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value


        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )


        self.wheel_cmd_pub = self.create_publisher(
            Twist,
            '/wheel_cmd',
            10
        )


        self.cmd_vel_filtered_pub = self.create_publisher(
            Twist,
            '/cmd_vel_filtered',
            10
        )



    def cmd_vel_callback(self, msg):

        # 限制速度
        linear_x = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))



        v_fl = linear_x - angular_z * (self.wheel_base / 2 + self.track_width / 2)
        v_fr = linear_x + angular_z * (self.wheel_base / 2 + self.track_width / 2)
        v_rl = linear_x - angular_z * (self.wheel_base / 2 + self.track_width / 2)
        v_rr = linear_x + angular_z * (self.wheel_base / 2 + self.track_width / 2)


        wheel_cmd = Twist()
        wheel_cmd.linear.x = v_fl
        wheel_cmd.linear.y = v_fr
        wheel_cmd.linear.z = v_rl
        wheel_cmd.angular.x = v_rr


        self.wheel_cmd_pub.publish(wheel_cmd)


        filtered_cmd = Twist()
        filtered_cmd.linear.x = linear_x
        filtered_cmd.angular.z = angular_z
        self.cmd_vel_filtered_pub.publish(filtered_cmd)



    def calculate_odometry(self, wheel_speeds, dt):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelTransformer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()