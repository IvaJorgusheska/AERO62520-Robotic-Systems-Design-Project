"""
================================================================================
COMPONENT: Navigation Controller Node (ActionClient Actuator)
================================================================================
Design Philosophy:
    Acts strictly as a "muscle" for the Master FSM. Merges robust ActionClient 
    implementation for real robots with a simplified "Dumb Actuator" architecture.
    
Interface Contract:
    - Subscribes: /goal_pose (PoseStamped) -> The target destination from FSM.
    - Publishes: /nav/goal_reached (Bool)  -> True when Nav2 completes the task.
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# ==============================================================================
# 1. NAVIGATION CONTROLLER NODE
# ==============================================================================
class NavControllerNode(Node):
    def __init__(self):
        # Enforce real system time for physical robot deployment
        super().__init__('nav_controller_node', 
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        
        # --- Initialize Nav2 Action Client ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 Action Server to become active (Real Robot Mode)...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 is active! Navigation actuator ready.')

        # --- Publishers & Subscribers ---
        # Listen to FSM for destination coordinates
        self.sub_goal = self.create_subscription(
            PoseStamped, 
            '/goal_pose', 
            self.goal_callback, 
            10
        )
        
        # Feedback to FSM upon successful arrival
        self.pub_fb = self.create_publisher(Bool, '/nav/goal_reached', 10)

        # --- Internal Execution State ---
        self.is_navigating = False      
        self.current_goal_handle = None 

    # ==========================================================================
    # CORE: GOAL EXECUTION & PREEMPTION
    # ==========================================================================
    def goal_callback(self, msg: PoseStamped):
        """
        Triggered when FSM commands a new destination.
        Automatically preempts any currently active Nav2 task.
        """
        self.get_logger().info(f"FSM commanded new goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        
        # Preempt current task if actively navigating
        if self.is_navigating and self.current_goal_handle:
            self.get_logger().info('Preempting current navigation task...')
            self.current_goal_handle.cancel_goal_async()

        # Reset timestamp to 0 to ensure immediate processing in real TF trees
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        
        # Package and dispatch the action goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        
        self.is_navigating = True
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    # ==========================================================================
    # CORE: ACTION CLIENT ASYNC CALLBACK CHAIN
    # ==========================================================================
    def goal_response_callback(self, future):
        """Verifies if the Nav2 server accepted the commanded goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2 server (Path blocked or invalid).')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted. Chassis is moving...')
        self.current_goal_handle = goal_handle
        
        # Await final navigation result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Processes the final result (Success, Cancelled, or Failed) and notifies FSM."""
        status = future.result().status
        self.is_navigating = False
        self.current_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Arrival Successful! Notifying FSM.')
            msg = Bool()
            msg.data = True
            self.pub_fb.publish(msg)
            
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation task was canceled (Likely preempted by new FSM goal).')
            
        else:
            self.get_logger().error(f'Navigation failed with status code: {status}')
            # On failure, no signal is sent. FSM will wait or eventually timeout.

def main(args=None):
    rclpy.init(args=args)
    node = NavControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()