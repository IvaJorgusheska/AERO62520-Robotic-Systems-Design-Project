import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import GripperState, GripperPose
from pymycobot import MyCobot280,PI_PORT,PI_BAUD
import time

arm=MyCobot280(PI_PORT,PI_BAUD)

init_coords = arm.get_coords()
arm.send_coords(init_coords,20,1)

print (init_coords)

class ManipulatorControlNode(Node):
    def __init__(self):
        super().__init__('manipulator_control_node')
        self.gripper_state_subscriber = self.create_subscription(
            msg_type=GripperState,
            topic='/arm/grasp_status',
            callback=self.gripper_state_subscriber_callback,
            qos_profile=1)

        self.gripper_pose_subscriber = self.create_subscription(
            msg_type=GripperPose,
            topic='/arm/grasp_pose',
            callback=self.gripper_pose_subscriber_callback,
            qos_profile=1)
        
    def gripper_state_subscriber_callback(self, msg: GripperState):
        self.get_logger().info(f"Received GripperState: grip={msg.grip}")
        if msg.grip:
            arm.set_gripper_state(0,100)
        if not msg.grip:
            arm.set_gripper_state(1,100)

    def gripper_pose_subscriber_callback(self, msg: GripperPose):
        self.get_logger().info(
        f"Received GripperPose: x={msg.x}, y={msg.y}, z={msg.z}, "
        f"roll={msg.roll}, pitch={msg.pitch}, yaw={msg.yaw}"
        )
        arm.send_coords([msg.x,msg.y,msg.z,msg.roll,msg.pitch,msg.yaw],20,1)

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        manipulator_control_node = ManipulatorControlNode()

        rclpy.spin(manipulator_control_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()