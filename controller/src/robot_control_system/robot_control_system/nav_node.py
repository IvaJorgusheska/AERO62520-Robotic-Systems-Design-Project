"""
=======================================================================================
Robot Navigation Microservice - Spatial Execution Engine
=======================================================================================
Design Philosophy:
    Acts as an autonomous navigation driver. It abstracts complex Nav2 behaviors 
    and exposes a simple boolean-based interface to the FSM. 

Key Feature: Multi-Topic Spatial Memory
    - [MODIFIED] Listens to separate vision topics: '/target_object_pose' and '/target_box_pose'.
    - Maintains a local cache of coordinates to allow immediate reaction when 
      the Vision node locks a pair.
    - [MODIFIED] Clears local memory upon receiving a '/status/reset_vision' signal 
      to ensure cycle-to-cycle consistency.

Interface Contract:
    - Moving=True, Going_to_box=False : Search/Approach the TARGET OBJECT.
    - Moving=True, Going_to_box=True  : Search/Approach the TARGET BOX.
    - Moving=False                   : Immediate Halt via Nav2 cancel.
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
        self.is_going_to_box = False # [MODIFIED] Renamed for interface consistency
        self.current_nav_intent = "IDLE" 
        
        # Spatial Memory Dictionary
        self.spatial_memory = {
            'target_object': None,  
            'target_box': None     
        }

        # Exploration Boundaries
        self.map_limit_x = (-2.0, 2.0)
        self.map_limit_y = (-2.0, 2.0)

        # === 3. Publishers (Feedback to FSM) ===
        self.pub_move_fb = self.create_publisher(Bool, '/move_feedback', 10)

        # === 4. Subscribers ===
        # Commands from FSM
        self.sub_moving = self.create_subscription(Bool, '/status/moving', self.moving_callback, 10)
        # [MODIFIED] Updated topic name to match FSM
        self.sub_to_box = self.create_subscription(Bool, '/status/going_to_box', self.going_to_box_callback, 10)
        # [MODIFIED] Added Reset subscriber to clear navigation memory
        self.sub_reset = self.create_subscription(Bool, '/status/reset_vision', self.reset_memory_callback, 10)
        # [MODIFIED] Split into separate topic subscriptions
        self.sub_obj_pose = self.create_subscription(
            PoseStamped, '/target_object_pose', self.object_pose_callback, 10)
        self.sub_box_pose = self.create_subscription(
            PoseStamped, '/target_box_pose', self.box_pose_callback, 10)

        # === 5. Core Control Engine ===
        self.control_timer = self.create_timer(0.5, self.control_loop)

    # =========================================================================
    # Callbacks (Input Processing)
    # =========================================================================
    
    def reset_memory_callback(self, msg):
        """[MODIFIED] Clears local pose cache when a cycle is finished."""
        if msg.data:
            self.get_logger().info('Clearing local navigation cache for new cycle.')
            self.spatial_memory['target_object'] = None
            self.spatial_memory['target_box'] = None

    def going_to_box_callback(self, msg):
        """[MODIFIED] Updates the target destination flag."""
        self.is_going_to_box = msg.data

    def moving_callback(self, msg):
        self.is_moving_enabled = msg.data
        if not self.is_moving_enabled:
            self.get_logger().info('Halt signal. Canceling tasks.')
            self.navigator.cancelTask()
            self.current_nav_intent = "IDLE"

    # [MODIFIED] Replaced unified vision callback with specific topic handlers
    def object_pose_callback(self, msg):
        """Memorizes the object location and interrupts exploration if searching."""
        self.spatial_memory['target_object'] = msg
        if self.is_moving_enabled and not self.is_going_to_box and self.current_nav_intent == "EXPLORING":
            self.get_logger().info('Object found! Interrupting exploration.')
            self.navigator.cancelTask()
            self.current_nav_intent = "IDLE" # Force re-evaluation in loop

    def box_pose_callback(self, msg):
        """Memorizes the box location and interrupts exploration if searching."""
        self.spatial_memory['target_box'] = msg
        if self.is_moving_enabled and self.is_going_to_box and self.current_nav_intent == "EXPLORING":
            self.get_logger().info('Box found! Interrupting exploration.')
            self.navigator.cancelTask()
            self.current_nav_intent = "IDLE" # Force re-evaluation in loop

    # =========================================================================
    # Main Control Loop
    # =========================================================================
    def control_loop(self):
        if not self.is_moving_enabled:
            return 

        # Determine current target coordinate based on FSM mode
        target_key = 'target_box' if self.is_going_to_box else 'target_object'
        target_intent = "APPROACHING_BOX" if self.is_going_to_box else "APPROACHING_OBJ"
        
        # 1. Coordinate Known: Navigate to target
        if self.spatial_memory[target_key] is not None:
            if self.current_nav_intent != target_intent:
                self.get_logger().info(f'[Action] Navigating to {target_key}.')
                self.current_nav_intent = target_intent
                # Refresh timestamp for Nav2
                self.spatial_memory[target_key].header.stamp = self.navigator.get_clock().now().to_msg()
                self.navigator.goToPose(self.spatial_memory[target_key])
            else:
                self.check_arrival(target_intent)
        
        # 2. Coordinate Unknown: Random Exploration
        else:
            if self.current_nav_intent != "EXPLORING":
                self.get_logger().info(f'Target {target_key} unknown. Exploring arena...')
                self.current_nav_intent = "EXPLORING"
                self.send_random_waypoint()
            elif self.navigator.isTaskComplete():
                self.send_random_waypoint()

    # =========================================================================
    # Helpers
    # =========================================================================
    def check_arrival(self, intent_type):
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Arrival Successful: {intent_type}')
                msg = Bool(); msg.data = True
                self.pub_move_fb.publish(msg)
                self.current_nav_intent = "IDLE" 
            elif result == TaskResult.FAILED:
                self.get_logger().error('Navigation failed.')

    def send_random_waypoint(self):
        x = random.uniform(self.map_limit_x[0], self.map_limit_x[1])
        y = random.uniform(self.map_limit_y[0], self.map_limit_y[1])
        # Random yaw for orientation (optional, can be set to 0 for simplicity)
        yaw = random.uniform(-math.pi, math.pi)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        # Convert yaw to quaternion (assuming roll=pitch=0)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.navigator.goToPose(goal)

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