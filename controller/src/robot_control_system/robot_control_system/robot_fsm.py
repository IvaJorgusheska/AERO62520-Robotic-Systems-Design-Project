"""
=======================================================================================
Robot Finite State Machine (FSM) Node - Central Controller
=======================================================================================
Design Philosophy:
    This node serves as the central "brain" (FSM/PLC) of the robotic system, 
    adhering strictly to an event-driven and signal-interlock architecture.
    It does not process complex navigation algorithms or vision matrices; instead, 
    it orchestrates the mission by exchanging Boolean (True/False) signals with 
    sub-system nodes.

System State Machine Flow Diagram:
    [0] INIT (Hardware Self-Check)
          | (All Sub-systems Online)
          v
    [1] SEARCH (Global Patrol for Target)
          | (/detected_object == True)
          v
    [2] MOVE_TO_TARGET (Navigate to Target)
          | (/move_feedback == True)
          v
    [3] GRASP (Manipulator Pick)
          | (/manipulator_feedback == True)
          v
    [4] RETURN (Navigate back to Base)
          | (/move_feedback == True)
          v
    [5] DROP (Manipulator Place)
          | (/manipulator_feedback == True)
          +---> Completed 3 cycles? ---> [Exit Mission]
          | (Else)
          +---> [Loop back to 1 SEARCH]

Output Control Matrix:
    | State | Moving | Arm_Active | Returning | Description
    |-------|--------|------------|-----------|-------------------------
    | SEARCH|  True  |   False    |   False   | Chassis explores, arm stowed.
    | MOVE  |  True  |   False    |   False   | Chassis approaches target.
    | GRASP |  False |   True     |   False   | Chassis halts, arm picks target.
    | RETURN|  True  |   False    |   True    | Chassis returns to base.
    | DROP  |  False |   True     |   True    | Chassis halts, arm places target.
=======================================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import sys

# Import custom message for error handling
# Ensure 'my_robot_interfaces' is built and sourced in your colcon workspace
from my_robot_interfaces.msg import ErrorStatus

# === State Machine Constants (Enumerations) ===
STATE_INIT = 0           # Initialization and hardware verification
STATE_SEARCH = 1         # Target searching phase
STATE_MOVE_TO_TARGET = 2 # Approaching target phase
STATE_GRASP = 3          # Object grasping phase
STATE_RETURN = 4         # Returning to base phase
STATE_DROP = 5           # Object placement phase

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # === Internal Variables Initialization ===
        self.current_state = STATE_INIT
        self.cycle_count = 0  # Counter for completed pick-and-place cycles (n)
        self.max_cycles = 3   # Terminate mission after 3 successful cycles

        # Hardware readiness dictionary for startup interlock
        self.hardware_ready = {'camera': False, 'nav': False, 'arm': False}
        self.latest_object_pose = None # Cache for the latest target 6D pose

        # === Action Publishers ===
        # Note: Commands are published ONLY on state transitions, not at high frequency.
        self.pub_moving = self.create_publisher(Bool, '/status/moving', 10)         # Controls chassis movement
        self.pub_arm = self.create_publisher(Bool, '/status/arm_active', 10)        # Controls manipulator activation
        self.pub_return = self.create_publisher(Bool, '/status/returning', 10)      # Indicates homing/placement mode

        # === Sensor / Feedback Subscribers ===
        # 1. Vision Node Feedback (Triggers: Search -> Move)
        self.sub_detect = self.create_subscription(
            Bool, '/detected_object', self.detect_callback, 10)
            
        # Receive the latest 6D pose (Logged for data integrity, not used for FSM logic)
        self.sub_object_pose = self.create_subscription(
            PoseStamped, '/object_pose', self.object_pose_callback,10)

        # 2. Navigation Node Feedback (Triggers: Move -> Grasp AND Return -> Drop)
        self.sub_move_fb = self.create_subscription(
            Bool, '/move_feedback', self.move_feedback_callback, 10)
            
        # 3. Manipulator Node Feedback (Triggers: Grasp -> Return AND Drop -> Search)
        self.sub_manip_fb = self.create_subscription(
            Bool, '/manipulator_feedback', self.manip_feedback_callback, 10)
            
        # 4. Global Error Monitoring (E-Stop Trigger)
        self.sub_error = self.create_subscription(
            ErrorStatus, '/status_error', self.error_callback, 10)
            
        self.get_logger().info(f'Node initialized. State: INIT. Cycle: {self.cycle_count}')
        
        # 5. Hardware Readiness Polling Timer (1Hz frequency)
        self.init_timer = self.create_timer(1.0, self.check_hardware_readiness)

    def check_hardware_readiness(self):
        """
        Startup self-check sequence.
        Monitors the ROS graph for required topic publishers to ensure sub-systems are online.
        Transitions to the SEARCH state only when Camera, Navigation, and Manipulator nodes are ready.
        """
        # Cancel timer to save computational resources if initialization is complete
        if self.current_state != STATE_INIT:
            self.init_timer.cancel()
            return
            
        # 1. Verify Camera Node
        if not self.hardware_ready['camera']:
            if self.count_publishers('/detected_object') > 0:
                self.hardware_ready['camera'] = True
                self.get_logger().info('[Check] Camera Node: ONLINE ✅')
                
        # 2. Verify Navigation Node
        if not self.hardware_ready['nav']:
            if self.count_publishers('/move_feedback') > 0:
                self.hardware_ready['nav'] = True
                self.get_logger().info('[Check] Navigation Node: ONLINE ✅')
                
        # 3. Verify Manipulator Node
        if not self.hardware_ready['arm']:
            if self.count_publishers('/manipulator_feedback') > 0:
                self.hardware_ready['arm'] = True
                self.get_logger().info('[Check] Manipulator Node: ONLINE ✅')
                
        # Transition to SEARCH state once all hardware interlocks are cleared
        if all(self.hardware_ready.values()):
            self.get_logger().info('='*40)
            self.get_logger().info('>>> ALL SYSTEMS GO! Starting Mission... <<<')
            self.get_logger().info('='*40)
            self.current_state = STATE_SEARCH
            
            # Initial command: Engage chassis, stow arm, standard exploration mode
            self.publish_state_flags(moving=True, arm=False, returning=False)
            self.init_timer.cancel() # Destroy timer post-initialization

    def publish_state_flags(self, moving: bool, arm: bool, returning: bool):
        """
        Unified status flag publisher.
        Encapsulates message construction and ensures atomic publication during state transitions.
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
        Global Emergency Stop (E-Stop) handler.
        Listens to the /status_error topic and forcibly terminates the node process upon severe faults.
        """
        if msg.error_state:
            self.get_logger().fatal(f'CRITICAL ERROR RECEIVED. Error Number: {msg.error_number}')
            self.get_logger().fatal('Terminating Node due to hardware fault...')
            raise SystemExit # Triggers a clean shutdown in the main execution block

    def detect_callback(self, msg):
        """
        Camera vision feedback handler.
        Logic: Triggers when the system is actively searching and vision detects a valid target.
        Transition: SEARCH (1) -> MOVE_TO_TARGET (2)
        """
        if not msg.data:
            return

        # Interlock to prevent false triggers during grasping or returning phases
        if self.current_state == STATE_SEARCH:
            self.get_logger().info('Event: Object Detected -> Switching to MOVE state')
            self.current_state = STATE_MOVE_TO_TARGET
            
            # Command update: Chassis approaches target, arm remains stowed
            self.publish_state_flags(moving=True, arm=False, returning=False)

    def object_pose_callback(self, msg):
        """
        Target pose cache handler.
        Continuously updates the 3D coordinates for potential downstream data logging or telemetry.
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
        Navigation arrival feedback handler.
        Evaluates current state to determine the appropriate post-navigation action (Grasp or Drop).
        """
        if not msg.data:
            return

        # Scenario 1: Arrived at the target object location
        if self.current_state == STATE_MOVE_TO_TARGET:
            self.get_logger().info('Event: Arrived at Target -> Switching to GRASP state')
            self.current_state = STATE_GRASP
            
            # Command update: Halt chassis, activate arm for picking
            self.publish_state_flags(moving=False, arm=True, returning=False)

        # Scenario 2: Arrived back at the home base
        elif self.current_state == STATE_RETURN:
            self.get_logger().info('Event: Returned to Base -> Switching to DROP state')
            self.current_state = STATE_DROP
            
            # Command update: Halt chassis, activate arm with 'returning' flag to indicate placement
            self.publish_state_flags(moving=False, arm=True, returning=True)

    def manip_feedback_callback(self, msg):
        """
        Manipulator action completion feedback handler.
        Evaluates current state to determine the next system-level action after arm operation.
        """
        if not msg.data:
            return

        # Scenario 1: Object successfully grasped
        if self.current_state == STATE_GRASP:
            self.get_logger().info('Event: Object Grasped -> Switching to RETURN state')
            self.current_state = STATE_RETURN
            
            # Command update: Engage chassis, stow arm, set homing flag
            self.publish_state_flags(moving=True, arm=False, returning=True)

        # Scenario 2: Object successfully dropped/placed
        elif self.current_state == STATE_DROP:
            self.cycle_count += 1
            self.get_logger().info(f'Event: Object Dropped. Cycle count: {self.cycle_count}')

            # Evaluate mission termination criteria
            if self.cycle_count >= self.max_cycles:
                self.get_logger().info('Task Completed (n=3). Shutting down node.')
                raise SystemExit # Clean exit upon successful mission completion
            else:
                self.get_logger().info('-> Restarting cycle: Switching to SEARCH state')
                self.current_state = STATE_SEARCH
                
                # Reset flags for a new exploration cycle
                self.publish_state_flags(moving=True, arm=False, returning=False)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        # Handles intentional termination (mission complete or E-Stop)
        rclpy.shutdown()
    except KeyboardInterrupt:
        # Handles user-initiated Ctrl+C
        pass

if __name__ == '__main__':
    main()