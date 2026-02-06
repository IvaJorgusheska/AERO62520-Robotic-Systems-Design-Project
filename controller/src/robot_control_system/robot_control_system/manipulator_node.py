import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs # Required for transform operations
import time

# Ensure you have built the interface package from previous steps
from my_robot_interfaces.msg import ObjectTarget

# === 1. Define Object Class ===
class TargetObject:
    def __init__(self):
        self.name = None
        self.color = None
        # Position and Orientation (x, y, z, qx, qy, qz, qw)
        self.pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 
                     'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}

# State Constants
STATE_IDLE = 0
STATE_GRASP = 1
STATE_DROP = 2

class ManipulatorControlNode(Node):
    def __init__(self):
        super().__init__('manipulator_control_node')

        # === 2. Initialization ===
        # Instantiate Object class
        self.target_object = TargetObject()
        
        # Grasp mark (False = Empty hand, True = Holding object)
        self.grasp_mark = False
        
        # Define Poses (Using dict for simplicity in this example)
        # Initial Pose (Home)
        self.pose_initial = {'x': 0.2, 'y': 0.0, 'z': 0.4, 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1}
        # Transport Pose (Drop location)
        self.pose_transport = {'x': -0.2, 'y': 0.3, 'z': 0.2, 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1}

        # State Machine Initialization
        self.current_state = STATE_IDLE
        self.startup_sequence_done = False # To run initial move once

        # Variable to store latest object detail message
        self.latest_object_name = None
        self.latest_object_color = None

        # === ROS Communications ===
        # Publishers
        self.pub_feedback = self.create_publisher(Bool, '/manipulator_feedback', 10)

        # Subscribers
        self.sub_active = self.create_subscription(
            Bool, '/status/arm_active', self.active_callback, 10)
        
        self.sub_detail = self.create_subscription(
            ObjectTarget, '/object_detail', self.detail_callback, 10)

        # TF Buffer & Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer (Main Loop - 10Hz)
        self.timer = self.create_timer(0.1, self.main_loop)
        
        self.get_logger().info('Manipulator Node Initialized.')

    # === Callbacks ===
    def active_callback(self, msg):
        """
        Callback for /status/arm_active.
        Triggers state transition based on grasp_mark.
        """
        if msg.data:
            if self.current_state == STATE_IDLE:
                if not self.grasp_mark:
                    self.get_logger().info('Command Received: Switch to GRASP state')
                    self.current_state = STATE_GRASP
                else:
                    self.get_logger().info('Command Received: Switch to DROP state')
                    self.current_state = STATE_DROP

    def detail_callback(self, msg):
        """
        Callback for /object_detail.
        Updates the target name to look for.
        """
        if msg.observe_state:
            self.latest_object_name = msg.object_name
            self.latest_object_color = msg.color

    # === 5. Arm Move Function (Pseudo) ===
    def move_arm_to_pose(self, pose_dict):
        """
        Pseudocode function to move the arm.
        Input: Dictionary with position and orientation.
        """
        self.get_logger().info(f'--- Moving Arm to: {pose_dict} ---')
        
        # [Hardware Driver Code Would Go Here]
        # e.g., driver.set_target(pose_dict)
        # while driver.is_moving(): time.sleep(0.1)
        
        # Simulating movement delay
        time.sleep(1.0) 
        self.get_logger().info('--- Movement Complete ---')

    # === 6. Gripper Function (Pseudo) ===
    def set_gripper(self, is_open: bool):
        """
        Pseudocode function to control gripper.
        Input: True to Open, False to Close.
        """
        state_str = "OPEN" if is_open else "CLOSED"
        self.get_logger().info(f'--- Gripper Actuating: {state_str} ---')
        
        # [Hardware Driver Code Would Go Here]
        # e.g., gripper.set_state(is_open)
        
        time.sleep(0.5) # Simulating actuation time

    # === 7. Coordinate Transform Function ===
    def transform_to_manipulator_base(self, target_frame_name):
        """
        Reads TF tree to find target_frame relative to base_link,
        then transforms it to be relative to manipulator_base.
        
        Assumptions:
        1. We look up transform: base_link -> target_frame
        2. We assume we need the result in 'manipulator_base' frame
        """
        try:
            # Step A: Get Object pose relative to base_link (from TF tree)
            # lookup_transform(target_frame, source_frame) -> base_link to object
            t_base_obj = self.tf_buffer.lookup_transform(
                'base_link',
                target_frame_name,
                rclpy.time.Time())

            # Create a PoseStamped from the transform result
            pose_in_base = PoseStamped()
            pose_in_base.header.frame_id = 'base_link'
            pose_in_base.pose.position.x = t_base_obj.transform.translation.x
            pose_in_base.pose.position.y = t_base_obj.transform.translation.y
            pose_in_base.pose.position.z = t_base_obj.transform.translation.z
            pose_in_base.pose.orientation = t_base_obj.transform.rotation

            # Step B: Transform from base_link to manipulator_base
            # We need the transform: manipulator_base -> base_link
            t_manip_base = self.tf_buffer.lookup_transform(
                'manipulator_base',
                'base_link',
                rclpy.time.Time())
            
            # Execute transform
            pose_in_manip = tf2_geometry_msgs.do_transform_pose(pose_in_base, t_manip_base)
            
            # Convert back to dict for our internal logic
            result_pose = {
                'x': pose_in_manip.pose.position.x,
                'y': pose_in_manip.pose.position.y,
                'z': pose_in_manip.pose.position.z,
                'qx': pose_in_manip.pose.orientation.x,
                'qy': pose_in_manip.pose.orientation.y,
                'qz': pose_in_manip.pose.orientation.z,
                'qw': pose_in_manip.pose.orientation.w
            }
            return result_pose

        except Exception as e:
            self.get_logger().error(f'Transform error: {str(e)}')
            return None

    # === 8. Main Loop ===
    def main_loop(self):
        # Startup Logic: Move to Initial Pose once
        if not self.startup_sequence_done:
            self.get_logger().info('Startup: Moving to Initial Pose...')
            self.move_arm_to_pose(self.pose_initial)
            self.startup_sequence_done = True
            return

        # State Machine Logic
        if self.current_state == STATE_GRASP:
            self.execute_grasp_sequence()
            
        elif self.current_state == STATE_DROP:
            self.execute_drop_sequence()

    def execute_grasp_sequence(self):
        """
        Logic for Grasp State
        """
        if self.latest_object_name is None:
            self.get_logger().warn('Waiting for object name...')
            return

        self.get_logger().info(f'Starting Grasp Sequence for: {self.latest_object_name}')

        # 1. Update Object Instance attributes
        self.target_object.name = self.latest_object_name
        self.target_object.color = self.latest_object_color
        
        # 2. Get Coordinates (Transform from base_link to manipulator_base)
        # Look up the frame named after the object (e.g. "Apple")
        target_pose = self.transform_to_manipulator_base(self.target_object.name)
        
        if target_pose:
            # Update Object position
            self.target_object.pose = target_pose
            
            # 3. Move Arm
            self.move_arm_to_pose(self.target_object.pose)
            
            # 4. Close Gripper (False = Close)
            self.set_gripper(False)
            self.move_arm_to_pose(self.pose_transport)# move arm to transport pose
            
            # 5. Send Feedback
            msg = Bool()
            msg.data = True
            self.pub_feedback.publish(msg)
            
            # 6. Update Mark and State
            self.grasp_mark = True
            self.current_state = STATE_IDLE
            self.get_logger().info('Grasp Complete. State -> IDLE')
        else:
             self.get_logger().error('Could not determine target pose.')
             # Reset to IDLE to avoid infinite loop of errors
             self.current_state = STATE_IDLE

    def execute_drop_sequence(self):
        """
        Logic for Drop/Placement State
        """
        self.get_logger().info('Starting Drop Sequence...')

        # 1. Move Arm to Transport/Drop Pose
        self.move_arm_to_pose(self.pose_transport)
        
        # 2. Open Gripper (True = Open)
        self.set_gripper(True)
        
        # 3. Send Feedback
        msg = Bool()
        msg.data = True
        self.pub_feedback.publish(msg)
        
        # 4. Update Mark and State
        self.grasp_mark = False
        self.current_state = STATE_IDLE
        self.get_logger().info('Drop Complete. State -> IDLE')

def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()