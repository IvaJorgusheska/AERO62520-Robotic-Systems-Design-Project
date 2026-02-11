import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import sys

# Import custom message for error handling
# Ensure 'my_robot_interfaces' is built and sourced in your workspace
from my_robot_interfaces.msg import ErrorStatus

# === State Constants ===
STATE_INIT = 0
STATE_SEARCH = 1
STATE_MOVE_TO_TARGET = 2
STATE_GRASP = 3
STATE_RETURN = 4
STATE_DROP = 5

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # === Internal Variables ===
        self.current_state = STATE_INIT
        self.cycle_count = 0  # Counter for completed tasks (n)
        self.max_cycles = 3   # Stop after 3 cycles

        self.hardware_ready = {'camera': False, 'nav': False, 'arm': False}

        self.latest_object_pose = None


        # === Publishers ===
        # Publishing status flags only on state transitions
        # 'latched' behavior is often emulated by QoS, but here we simply publish once per event.
        self.pub_moving = self.create_publisher(Bool, '/status/moving', 10)
        self.pub_arm = self.create_publisher(Bool, '/status/arm_active', 10)
        self.pub_return = self.create_publisher(Bool, '/status/returning', 10)


        # === Subscribers ===
        # 1. Object Detection (Triggers Search -> Move)
        self.sub_detect = self.create_subscription(
            Bool, '/detected_object', self.detect_callback, 10)

        # Latest object pose (not used for state transitions but used for manipulation and navigation)
        self.sub_object_pose = self.create_subscription(
            PoseStamped, '/object_pose', self.object_pose_callback,10)

        # 2. Movement Feedback (Triggers Move -> Grasp AND Return -> Drop)
        self.sub_move_fb = self.create_subscription(
            Bool, '/move_feedback', self.move_feedback_callback, 10)
        # 3. Manipulator Feedback (Triggers Grasp -> Return AND Drop -> Search)
        self.sub_manip_fb = self.create_subscription(
            Bool, '/manipulator_feedback', self.manip_feedback_callback, 10)
        # 4. Error Status Monitoring
        self.sub_error = self.create_subscription(
            ErrorStatus, '/status_error', self.error_callback, 10)
        self.get_logger().info(f'Node initialized. State: INIT. Cycle: {self.cycle_count}')
        # 5. Hardware Readiness Check Timer (Polls every 1 second)
        self.init_timer = self.create_timer(1.0, self.check_hardware_readiness)

    def check_hardware_readiness(self):
        """
        Periodically checks if all hardware nodes are online.
        Only transitions to SEARCH state when everyone is ready.
        """
        if self.current_state != STATE_INIT:
            self.init_timer.cancel()
            return
        # 1. /detected_object Publisher indicates Camera Node is online
        if not self.hardware_ready['camera']:
            if self.count_publishers('/detected_object') > 0:
                self.hardware_ready['camera'] = True
                self.get_logger().info('[Check] Camera Node: ONLINE ✅')
        # 2. /move_feedback Publisher indicates Navigation Node is online
        if not self.hardware_ready['nav']:
            if self.count_publishers('/move_feedback') > 0:
                self.hardware_ready['nav'] = True
                self.get_logger().info('[Check] Navigation Node: ONLINE ✅')
        # 3. /manipulator_feedback Publisher indicates Manipulator Node is online
        if not self.hardware_ready['arm']:
            if self.count_publishers('/manipulator_feedback') > 0:
                self.hardware_ready['arm'] = True
                self.get_logger().info('[Check] Manipulator Node: ONLINE ✅')
        # If all hardware is ready, transition to SEARCH state
        if all(self.hardware_ready.values()):
            self.get_logger().info('='*40)
            self.get_logger().info('>>> ALL SYSTEMS GO! Starting Mission... <<<')
            self.get_logger().info('='*40)
            self.current_state = STATE_SEARCH
            self.publish_state_flags(moving=True, arm=False, returning=False)
            self.init_timer.cancel()

    def publish_state_flags(self, moving: bool, arm: bool, returning: bool):
        """
        Helper function to publish all status flags at once.
        This is called only when a state transition occurs.
        """
        msg_moving = Bool()
        msg_moving.data = moving
        self.pub_moving.publish(msg_moving)

        msg_arm = Bool()
        msg_arm.data = arm
        self.pub_arm.publish(msg_arm)

        msg_return = Bool()
        msg_return.data = returning
        self.pub_return.publish(msg_return)

        self.get_logger().info(f'Status Published -> Moving: {moving}, Arm: {arm}, Returning: {returning}')

    def error_callback(self, msg):
        """
        Callback for /status_error.
        Terminates the program if error_state is True.
        """
        # Accessing fields using snake_case (standard ROS 2 convention)
        if msg.error_state:
            self.get_logger().fatal(f'CRITICAL ERROR RECEIVED. Error Number: {msg.error_number}')
            self.get_logger().fatal('Terminating Node due to error...')
            # Clean exit
            raise SystemExit

    def detect_callback(self, msg):
        """
        Callback for /detected_object.
        Transition: Search (1) -> Move (2)
        """
        if not msg.data:
            return

        if self.current_state == STATE_SEARCH:
            self.get_logger().info('Event: Object Detected -> Switching to MOVE state')
            self.current_state = STATE_MOVE_TO_TARGET
            
            # Logic: Moving=True, Arm=False, Return=False
            self.publish_state_flags(moving=True, arm=False, returning=False)

    def object_pose_callback(self, msg):
        """
        Stores the latest detected object pose.
        """
        self.latest_object_pose = msg
        self.get_logger().debug(
            f"Received object pose: "
            f"x={msg.pose.position.x:.2f}, "
            f"y={msg.pose.position.y:.2f}, "
            f"z={msg.pose.position.z:.2f}"
        )


    def move_feedback_callback(self, msg):
        """
        Callback for /move_feedback.
        Handles two transitions:
        1. Move (2) -> Grasp (3)
        2. Return (4) -> Drop (5)
        """
        if not msg.data:
            return

        # Transition 1: Move -> Grasp
        if self.current_state == STATE_MOVE_TO_TARGET:
            self.get_logger().info('Event: Arrived at Target -> Switching to GRASP state')
            self.current_state = STATE_GRASP
            
            # Logic: Moving=False, Arm=True, Return=False
            self.publish_state_flags(moving=False, arm=True, returning=False)

        # Transition 2: Return -> Drop
        elif self.current_state == STATE_RETURN:
            self.get_logger().info('Event: Returned to Base -> Switching to DROP state')
            self.current_state = STATE_DROP
            
            # Logic: Moving=False, Arm=True, Return=True
            self.publish_state_flags(moving=False, arm=True, returning=True)

    def manip_feedback_callback(self, msg):
        """
        Callback for /manipulator_feedback.
        Handles two transitions:
        1. Grasp (3) -> Return (4)
        2. Drop (5) -> Search (1) [Increments Counter]
        """
        if not msg.data:
            return

        # Transition 1: Grasp -> Return
        if self.current_state == STATE_GRASP:
            self.get_logger().info('Event: Object Grasped -> Switching to RETURN state')
            self.current_state = STATE_RETURN
            
            # Logic: Moving=True, Arm=False, Return=True
            self.publish_state_flags(moving=True, arm=False, returning=True)

        # Transition 2: Drop -> Search (Loop or End)
        elif self.current_state == STATE_DROP:
            self.cycle_count += 1
            self.get_logger().info(f'Event: Object Dropped. Cycle count: {self.cycle_count}')

            if self.cycle_count >= self.max_cycles:
                self.get_logger().info('Task Completed (n=3). Shutting down node.')
                raise SystemExit
            else:
                self.get_logger().info('-> Restarting cycle: Switching to SEARCH state')
                self.current_state = STATE_SEARCH
                
                # Logic: Moving=True, Arm=False, Return=False
                self.publish_state_flags(moving=True, arm=False, returning=False)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        # Handles the intentional exit from counting or error
        rclpy.shutdown()
    except KeyboardInterrupt:
        # Handles Ctrl+C
        pass

if __name__ == '__main__':
    main()