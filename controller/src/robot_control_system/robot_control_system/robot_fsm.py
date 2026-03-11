"""
================================================================================
COMPONENT: Main Controller Node (Master Logic & FSM)
================================================================================
1. STATE MACHINE FLOW (FSM)
--------------------------------------------------------------------------------
[0] INIT: Wait for hardware initialization -> [1] SEARCH.
[1] SEARCH: Chassis explores. Vision publishes 'camera_link' coords, master transforms 
            and stores 'map' coords. If object-box pair found -> [2] MOVE_TO_OBJECT.
[2] MOVE_TO_OBJECT: Master sends object 'map' pose to Nav -> wait -> [3] GRASP.
[3] GRASP: Master transforms object 'map' pose to current 'camera_link' and sends to arm. 
           Wait 5s. Check vision timestamp. Success -> [4] MOVE_TO_BOX. Fail -> Retry.
[4] MOVE_TO_BOX: Master sends box 'map' pose to Nav -> wait -> [5] DROP.
[5] DROP: Master transforms box 'map' pose to current 'camera_link' and sends to arm. 
          Wait 3s. Cycle counter++. If < 3 -> [1] SEARCH. Else -> Shutdown.

2. OUTPUT CONTROL MATRIX (Pure Boolean)
--------------------------------------------------------------------------------
| STATE            | NAV_EXPLORE | NAV_GOTO | ARM_GRASP | ARM_DROP |
|------------------|-------------|----------|-----------|----------|
| [0] INIT         | False       | False    | False     | False    |
| [1] SEARCH       | True        | False    | False     | False    |
| [2] MOVE_OBJECT  | False       | True     | False     | False    |
| [3] GRASP        | False       | False    | True      | False    |
| [4] MOVE_BOX     | False       | True     | False     | False    |
| [5] DROP         | False       | False    | False     | True     |
================================================================================
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs 

# Custom interfaces
from my_robot_interfaces.msg import ObjectTarget, GripperState, GripperPose

# ==============================================================================
# STATE MACHINE CONSTANTS
# ==============================================================================
STATE_INIT           = 0           
STATE_SEARCH         = 1         
STATE_MOVE_TO_OBJECT = 2 
STATE_GRASP          = 3          
STATE_MOVE_TO_BOX    = 4    
STATE_DROP           = 5

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        
        self.current_state = STATE_INIT
        self.cycle_count = 0  
        self.max_cycles = 3   
        
        # Unified tracking: Master ONLY stores global 'map' coordinates
        self.map_hash = {}    
        self.active_target_color = "NONE" 
        
        self.state_start_time = self.get_clock().now()
        self.arm_action_duration = 5.0  
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.pub_nav_point   = self.create_publisher(PointStamped, '/nav/goal_point', 10)
        self.pub_nav_explore = self.create_publisher(Bool, '/nav/cmd_explore', 10)
        self.pub_arm_pose    = self.create_publisher(GripperPose, '/arm/grasp_pose', 10)
        self.pub_arm_grip    = self.create_publisher(GripperState, '/arm/grasp_status', 10) 
        
        # Subscribers
        self.sub_vision_target = self.create_subscription(ObjectTarget, '/detected_object', self.vision_target_callback, 10)
        self.sub_move_fb       = self.create_subscription(Bool, '/nav/goal_reached', self.move_feedback_callback, 10)
        
        # Output Control Matrix
        self.control_matrix = {
            STATE_INIT:           {"nav_explore": False, "nav_goto": False, "arm_grasp": False, "arm_drop": False},
            STATE_SEARCH:         {"nav_explore": True,  "nav_goto": False, "arm_grasp": False, "arm_drop": False},
            STATE_MOVE_TO_OBJECT: {"nav_explore": False, "nav_goto": True,  "arm_grasp": False, "arm_drop": False},
            STATE_GRASP:          {"nav_explore": False, "nav_goto": False, "arm_grasp": True,  "arm_drop": False},
            STATE_MOVE_TO_BOX:    {"nav_explore": False, "nav_goto": True,  "arm_grasp": False, "arm_drop": False},
            STATE_DROP:           {"nav_explore": False, "nav_goto": False, "arm_grasp": False, "arm_drop": True}
        }
        
        self.init_timer = self.create_timer(1.0, self.check_hardware_readiness)
        self.control_loop_timer = self.create_timer(0.1, self.execute_control_loop) 
        self.get_logger().info('Master Control Node Initialized. Entering INIT state.')

    # ==========================================================================
    # CORE: TIMER-DRIVEN LOGIC & REAL-TIME DATA DISPATCH
    # ==========================================================================
    def execute_control_loop(self):
        time_in_state = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9

        # --- [1] SEARCH ---
        if self.current_state == STATE_SEARCH:
            # Inline pair check: scan map_hash for a complete object-box pair
            for color, items in self.map_hash.items():
                if items.get('object') is not None and items.get('box') is not None:
                    self.active_target_color = color
                    self.transition_to_state(STATE_MOVE_TO_OBJECT, f"Pair found: {color.upper()}")
                    break

        # --- [2] & [4] Pass 'map' coordinates to Nav ---
        elif self.current_state in [STATE_MOVE_TO_OBJECT, STATE_MOVE_TO_BOX]:
            target_type = 'object' if self.current_state == STATE_MOVE_TO_OBJECT else 'box'
            if self.active_target_color in self.map_hash:
                target_pt_data = self.map_hash[self.active_target_color].get(target_type)
                if target_pt_data is not None:
                    self.pub_nav_point.publish(target_pt_data['msg'])

        # --- [3] GRASP Verify ---
        elif self.current_state == STATE_GRASP:
            if time_in_state > self.arm_action_duration:
                grab_failed = False
                
                # Verify if vision recently updated the 'map' coordinate for this object
                if self.active_target_color in self.map_hash:
                    obj_data = self.map_hash[self.active_target_color].get('object')
                    if obj_data is not None:
                        time_diff = (self.get_clock().now() - obj_data['timestamp']).nanoseconds / 1e9
                        if time_diff < 1.5:
                            grab_failed = True

                if grab_failed:
                    self.get_logger().warn(f"Vision still sees {self.active_target_color} object. Retrying Grasp...")
                    self.state_start_time = self.get_clock().now()
                    self.trigger_arm_action(is_grasp=True)
                else:
                    self.get_logger().info("Grasp Verified Successful!")
                    self.transition_to_state(STATE_MOVE_TO_BOX, "Heading to Target Box.")

        # --- [5] DROP Wait ---
        elif self.current_state == STATE_DROP:
            if time_in_state > 3.0:  
                self.cycle_count += 1
                completed_color = self.active_target_color
                
                if completed_color in self.map_hash: 
                    del self.map_hash[completed_color]
                
                self.active_target_color = "NONE"
                    
                if self.cycle_count >= self.max_cycles:
                    self.get_logger().info('ALL 3 CYCLES COMPLETED. MISSION ACCOMPLISHED.')
                    raise SystemExit 
                else:
                    self.transition_to_state(STATE_SEARCH, f"Cycle {self.cycle_count} done! Resuming Search.")

    # ==========================================================================
    # CORE: STATE TRANSITIONS & HARDWARE TRIGGERS
    # ==========================================================================
    def transition_to_state(self, new_state, log_msg=""):
        self.current_state = new_state
        self.state_start_time = self.get_clock().now()
        
        if log_msg:
            self.get_logger().info(f"--- STATE: {new_state} | {log_msg} ---")
            
        matrix = self.control_matrix[self.current_state]
        self.pub_nav_explore.publish(Bool(data=matrix["nav_explore"]))
        
        if matrix["arm_grasp"]:
            self.trigger_arm_action(is_grasp=True)
        elif matrix["arm_drop"]:
            self.trigger_arm_action(is_grasp=False)

    def trigger_arm_action(self, is_grasp: bool):
        """Transforms 'map' coordinate to current 'camera_link' and sends to arm."""
        if self.active_target_color not in self.map_hash: return
        
        target_type = 'object' if is_grasp else 'box'
        target_data = self.map_hash[self.active_target_color].get(target_type)
        
        if target_data is not None:
            try:
                # Get the latest transform from map to camera_link
                t = self.tf_buffer.lookup_transform('camera_link', 'map', rclpy.time.Time())
                cam_pt = tf2_geometry_msgs.do_transform_point(target_data['msg'], t)
                
                pose_msg = GripperPose()
                pose_msg.x = float(cam_pt.point.x)
                pose_msg.y = float(cam_pt.point.y)
                pose_msg.z = float(cam_pt.point.z)
                pose_msg.roll, pose_msg.pitch, pose_msg.yaw = 180.0, 0.0, 0.0
                self.pub_arm_pose.publish(pose_msg)
                
            except TransformException as ex:
                self.get_logger().warn(f"TF Error: Could not transform to camera_link: {ex}")

        grip_msg = GripperState()
        grip_msg.grip = is_grasp
        self.pub_arm_grip.publish(grip_msg)

    # ==========================================================================
    # LOGIC: VISION DATA PUMP & TF
    # ==========================================================================
    def vision_target_callback(self, msg: ObjectTarget):
        """Converts incoming 'camera_link' coordinates to 'map' and stores them."""
        color, item_type = msg.color, msg.name 
        if color == "unknown": return
            
        if color not in self.map_hash:
            self.map_hash[color] = {'object': None, 'box': None}
        
        pt = PointStamped()
        pt.header.frame_id = 'camera_link'
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x, pt.point.y, pt.point.z = msg.x, msg.y, msg.z
        
        try:
            t = self.tf_buffer.lookup_transform('map', 'camera_link', rclpy.time.Time())
            map_pt = tf2_geometry_msgs.do_transform_point(pt, t)
            self.map_hash[color][item_type] = {'msg': map_pt, 'timestamp': self.get_clock().now()}
        except TransformException:
            pass

    # ==========================================================================
    # LOGIC: NAVIGATION FEEDBACK
    # ==========================================================================
    def move_feedback_callback(self, msg: Bool):
        if not msg.data: return
        
        if self.current_state == STATE_MOVE_TO_OBJECT:
            self.transition_to_state(STATE_GRASP, "Arrived at Object. Initiating Grasp Sequence.")
        elif self.current_state == STATE_MOVE_TO_BOX:
            self.transition_to_state(STATE_DROP, "Arrived at Box. Initiating Drop Sequence.")

    # ==========================================================================
    # AUXILIARY: HARDWARE CHECK
    # ==========================================================================
    def check_hardware_readiness(self):
        if self.current_state != STATE_INIT:
            self.init_timer.cancel()
            return
            
        camera_ready = self.count_publishers('/detected_object') > 0
        nav_ready    = self.count_publishers('/nav/goal_point') > 0
        arm_ready    = self.count_subscribers('/arm/grasp_pose') > 0
            
        if camera_ready and nav_ready and arm_ready:
            self.get_logger().info('>>> ALL SYSTEMS GO! Hardware verified. <<<')
            self.init_timer.cancel() 
            self.transition_to_state(STATE_SEARCH, "Commencing Global Patrol.")

def main():
    rclpy.init()
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()