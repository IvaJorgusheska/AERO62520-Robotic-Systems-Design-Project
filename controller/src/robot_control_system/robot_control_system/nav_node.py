"""
=======================================================================================
Robot Navigation Microservice with Semantic Memory
=======================================================================================
Design Philosophy:
    Acts as an autonomous navigation driver. It abstracts complex Nav2 behaviors 
    and exposes a simple boolean-based interface to the FSM. 

Key Feature: Semantic Spatial Memory
    - Listens to a SINGLE vision topic: '/vision/detected_objects'.
    - Uses the 'frame_id' field as a semantic label ("target_object" or "colored_box") 
      before converting it back to the absolute 'map' frame for Nav2.
    - Memorizes drop-off zones (colored boxes) encountered during initial exploration, 
      eliminating the need for secondary searches when delivering the payload.

Interface Contract:
    - Moving=True, Returning=False : Search/Approach the TARGET OBJECT (White Box).
    - Moving=True, Returning=True  : Search/Approach the COLORED BOX (Drop-off).
    - Moving=False                 : Immediate Halt.
=======================================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import random
import math

class NavControllerNode(Node):
    def __init__(self):
        super().__init__('nav_controller_node')
        
        # === 1. Initialize Nav2 API ===
        self.navigator = BasicNavigator()
        self.get_logger().info('Waiting for Nav2 stack to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active! Navigation microservice ready.')

        # === 2. Internal State & Memory ===
        self.is_moving_enabled = False
        self.is_returning_mode = False
        self.current_nav_intent = "IDLE" 
        # Intents: IDLE, EXPLORING_FOR_OBJ, APPROACHING_OBJ, EXPLORING_FOR_BOX, APPROACHING_BOX
        
        # 🧠 Spatial Memory Dictionary
        self.spatial_memory = {
            'target_object': None,  # Pose of the white box + object
            'colored_box': None     # Pose of the colored drop-off box
        }

        # Define Exploration Boundaries (in meters w.r.t 'map' frame)
        # TODO: Adjust these limits based on your actual arena/map size
        self.map_limit_x = (-2.0, 2.0)
        self.map_limit_y = (-2.0, 2.0)

        # === 3. Publishers (Feedback to FSM) ===
        self.pub_move_fb = self.create_publisher(Bool, '/move_feedback', 10)

        # === 4. Subscribers ===
        # Commands from FSM
        self.sub_moving = self.create_subscription(Bool, '/status/moving', self.moving_callback, 10)
        self.sub_returning = self.create_subscription(Bool, '/status/returning', self.returning_callback, 10)
        
        # Unified Vision Topic
        self.sub_vision = self.create_subscription(
            PoseStamped, 
            '/vision/detected_objects', 
            self.vision_callback, 
            10
        )

        # === 5. Core Control Engine ===
        self.control_timer = self.create_timer(0.5, self.control_loop)

    # =========================================================================
    # Standard Nav2 Goal Generator
    # =========================================================================
    def create_nav2_goal(self, x: float, y: float, theta: float = 0.0) -> PoseStamped:
        """Converts 2D planar coordinates into a 3D Pose with Quaternion for Nav2."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0
        
        # Euler Yaw to Quaternion conversion
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_pose.pose.orientation.w = math.cos(theta / 2.0)
        return goal_pose

    # =========================================================================
    # Callbacks (Input Processing & Interrupts)
    # =========================================================================
    def returning_callback(self, msg):
        """Updates the returning flag based on FSM command."""
        self.is_returning_mode = msg.data

    def moving_callback(self, msg):
        """Handles master moving switch from FSM. Halts immediately if disabled."""
        self.is_moving_enabled = msg.data
        if not self.is_moving_enabled:
            self.get_logger().info('[Command] Halt signal received. Canceling active tasks.')
            self.navigator.cancelTask()
            self.current_nav_intent = "IDLE"

    def vision_callback(self, msg):
        """
        Processes unified vision topic. Uses the 'frame_id' hack to determine class.
        """
        # 1. Extract the semantic label injected by the Vision node
        object_class = msg.header.frame_id
        
        # 2. CRITICAL: Override frame_id back to 'map' so Nav2 accepts the coordinate
        msg.header.frame_id = 'map'

        # --- Category A: Target Object (White Box) ---
        if object_class == "target_object":
            if self.spatial_memory['target_object'] is None:
                self.get_logger().info('👁️ [Vision] Target Object acquired and memorized!')
                self.spatial_memory['target_object'] = msg
                
                # Dynamic Interrupt: If exploring for the object, approach it immediately
                if self.is_moving_enabled and not self.is_returning_mode:
                    self.get_logger().info('[Action] Interrupting exploration to approach Target Object!')
                    self.navigator.cancelTask() 
                    self.current_nav_intent = "APPROACHING_OBJ"
                    msg.header.stamp = self.navigator.get_clock().now().to_msg() # Update time
                    self.navigator.goToPose(msg)

        # --- Category B: Colored Box (Drop-off Zone) ---
        elif object_class == "colored_box":
            if self.spatial_memory['colored_box'] is None:
                self.get_logger().info('👁️ [Memory] Colored Box acquired and memorized in background!')
                self.spatial_memory['colored_box'] = msg
                
                # Dynamic Interrupt: If exploring for the drop-off box, approach it immediately
                if self.is_moving_enabled and self.is_returning_mode:
                    self.get_logger().info('[Action] Interrupting exploration to approach Colored Box!')
                    self.navigator.cancelTask() 
                    self.current_nav_intent = "APPROACHING_BOX"
                    msg.header.stamp = self.navigator.get_clock().now().to_msg() # Update time
                    self.navigator.goToPose(msg)
        else:
            self.get_logger().warning(f"Received unknown object class in frame_id: {object_class}")

    # =========================================================================
    # Main Control Loop (Decision Making & Execution)
    # =========================================================================
    def control_loop(self):
        """The central event loop evaluating FSM intents and dispatching Nav2 goals."""
        if not self.is_moving_enabled:
            return 

        # ---------------------------------------------------------------------
        # Phase 2: Deliver / Drop-off Mode (Returning = True)
        # ---------------------------------------------------------------------
        if self.is_returning_mode:
            # Check Memory: Do we know where the colored box is?
            if self.spatial_memory['colored_box'] is not None:
                if self.current_nav_intent != "APPROACHING_BOX":
                    self.get_logger().info('[Action] Memory Hit! Navigating to memorized Colored Box.')
                    self.current_nav_intent = "APPROACHING_BOX"
                    self.spatial_memory['colored_box'].header.stamp = self.navigator.get_clock().now().to_msg()
                    self.navigator.goToPose(self.spatial_memory['colored_box'])
                else:
                    self.check_arrival("APPROACHING_BOX")
            
            # Memory Miss: Initiate secondary exploration to find the colored box
            else:
                if self.current_nav_intent != "EXPLORING_FOR_BOX":
                    self.get_logger().info('[Action] Colored Box unknown. Initiating secondary EXPLORATION.')
                    self.current_nav_intent = "EXPLORING_FOR_BOX"
                    self.send_random_waypoint()
                elif self.navigator.isTaskComplete():
                    self.send_random_waypoint()

        # ---------------------------------------------------------------------
        # Phase 1: Search / Pick-up Mode (Returning = False)
        # ---------------------------------------------------------------------
        else:
            # Check Memory: Do we know where the target object is?
            if self.spatial_memory['target_object'] is not None:
                if self.current_nav_intent != "APPROACHING_OBJ":
                    self.get_logger().info('[Action] Navigating to Target Object.')
                    self.current_nav_intent = "APPROACHING_OBJ"
                    self.spatial_memory['target_object'].header.stamp = self.navigator.get_clock().now().to_msg()
                    self.navigator.goToPose(self.spatial_memory['target_object'])
                else:
                    self.check_arrival("APPROACHING_OBJ")
            
            # Memory Miss: Initiate primary exploration to find the target object
            else:
                if self.current_nav_intent != "EXPLORING_FOR_OBJ":
                    self.get_logger().info('[Action] Initiating primary EXPLORATION for Target Object.')
                    self.current_nav_intent = "EXPLORING_FOR_OBJ"
                    self.send_random_waypoint()
                elif self.navigator.isTaskComplete():
                    self.send_random_waypoint()

    # =========================================================================
    # Task Monitoring & Helper Functions
    # =========================================================================
    def check_arrival(self, intent_type):
        """Monitors execution status, notifies FSM, and manages memory lifecycle."""
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'✅ Arrived successfully ({intent_type})! Notifying FSM...')
                
                # Notify FSM to proceed (Triggers Grasp or Drop)
                msg = Bool()
                msg.data = True
                self.pub_move_fb.publish(msg)
                
                # Reset intent to IDLE
                self.current_nav_intent = "IDLE" 
                
                # Memory Cleanup: Erase the consumed memory to prepare for the next cycle (n=1,2,3)
                if intent_type == "APPROACHING_OBJ":
                    self.spatial_memory['target_object'] = None
                elif intent_type == "APPROACHING_BOX":
                    self.spatial_memory['colored_box'] = None
            
            elif result == TaskResult.FAILED:
                self.get_logger().error('❌ Navigation failed. Awaiting FSM resolution or retrying...')

    def send_random_waypoint(self):
        """Generates and dispatches a random exploration waypoint."""
        x = random.uniform(self.map_limit_x[0], self.map_limit_x[1])
        y = random.uniform(self.map_limit_y[0], self.map_limit_y[1])
        random_yaw = random.uniform(-math.pi, math.pi)
        
        explore_goal = self.create_nav2_goal(x, y, random_yaw)
        self.navigator.goToPose(explore_goal)
        self.get_logger().debug(f'Exploring -> next waypoint: x={x:.2f}, y={y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = NavControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.navigator.cancelTask()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()