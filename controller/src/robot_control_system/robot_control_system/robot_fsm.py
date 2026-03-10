"""
================================================================================
COMPONENT: Main Controller Node (Master Logic & FSM)
================================================================================
1. STATE MACHINE FLOW (FSM)
--------------------------------------------------------------------------------
[0] INIT:
    - Logic: Hardware self-check (Vision, Nav2, Arm, TF).
    - Transition: If All Online -> [1] SEARCH.

[1] SEARCH:
    - Logic: Chassis performs global patrol. Monitor HashMap for Object + Box pair.
    - Transition: If pair found -> Set CURRENT_TARGET_COLOR -> [2] MOVE_TO_OBJECT.

[2] MOVE_TO_OBJECT:
    - Logic: Publish CURRENT_TARGET_COLOR. Send 'map_link' object pose to Nav2.
    - Transition: If Navigation returns 'Goal Reached' -> [3] GRASP.

[3] GRASP:
    - Logic: Chassis halts. Send 'camera_link' relative pose to Arm for precise pick.
    - Transition: If Arm returns 'Grasp Success' -> [4] MOVE_TO_BOX.

[4] MOVE_TO_BOX:
    - Logic: Keep publishing color. Send matching Box 'map_link' pose to Nav2.
    - Transition: If Navigation returns 'Goal Reached' -> [5] DROP.

[5] DROP:
    - Logic: Chassis halts. Send 'camera_link' relative pose to Arm for precise place.
    - Transition: 
        - If Total Cycles == 3 -> [Exit Mission / Shutdown].
        - Else -> Reset CURRENT_TARGET_COLOR to 'NONE' -> [1] SEARCH.

2. OUTPUT CONTROL MATRIX (Enum/Constant Based)
--------------------------------------------------------------------------------
| STATE        | CHASSIS_CMD | ARM_TASK  | VISION_MODE  | REF_FRAME | TARGET_TYPE |
|--------------|-------------|-----------|--------------|-----------|-------------|
| [0] INIT     | IDLE        | RESET     | STANDBY      | NONE      | NONE        |
| [1] SEARCH   | EXPLORE     | STOW      | SCAN_ALL     | MAP       | ANY         |
| [2] MOVE_OBJ | NAVIGATE    | STOW      | LOCK_COLOR   | MAP       | OBJECT      |
| [3] GRASP    | BRAKE       | PICK_ACT  | LOCK_COLOR   | CAMERA    | OBJECT      |
| [4] MOVE_BOX | NAVIGATE    | HOLD_OBJ  | LOCK_COLOR   | MAP       | BOX         |
| [5] DROP     | BRAKE       | PLACE_ACT | LOCK_COLOR   | CAMERA    | BOX         |

* Note: Matrix values map to defined Enums/Constants in code to prevent typos.
================================================================================
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # Added to prevent do_transform_pose error
from enum import Enum
from my_robot_interfaces.msg import ErrorStatus
# NOTE: You might need a custom message for vision that includes color and type.

# ==============================================================================
# 1. STATE MACHINE CONSTANTS
# ==============================================================================
STATE_INIT           = 0           
STATE_SEARCH         = 1         
STATE_MOVE_TO_OBJECT = 2 
STATE_GRASP          = 3          
STATE_MOVE_TO_BOX    = 4    
STATE_DROP           = 5

# ==============================================================================
# 2. OUTPUT CONTROL MATRIX ENUMS
# ==============================================================================
class ChassisCmd(Enum):
    IDLE     = "IDLE"
    EXPLORE  = "EXPLORE"
    NAVIGATE = "NAVIGATE"
    BRAKE    = "BRAKE"

class ArmTask(Enum):
    RESET     = "RESET"
    STOW      = "STOW"
    HOLD_OBJ  = "HOLD_OBJ"
    PICK_ACT  = "PICK_ACT"
    PLACE_ACT = "PLACE_ACT"

class VisionMode(Enum):
    STANDBY    = "STANDBY"
    SCAN_ALL   = "SCAN_ALL"
    LOCK_COLOR = "LOCK_COLOR"

class RefFrame(Enum):
    NONE   = "NONE"
    MAP    = "map"         
    CAMERA = "camera_link" 

class TargetType(Enum):
    NONE   = "NONE"
    ANY    = "ANY"
    OBJECT = "OBJECT"
    BOX    = "BOX"

# ==============================================================================
# 3. MAIN CONTROLLER NODE
# ==============================================================================
class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        # --- Mission Variables ---
        self.current_state = STATE_INIT
        self.cycle_count = 0  
        self.max_cycles = 3   
        
        # --- System Status ---
        self.hardware_ready = {'camera': False, 'nav': False, 'arm': False}
        
        # --- Vision Memory (The Registry) ---
        self.color_registry = {} 
        self.active_target_color = "NONE" 
        self.locked_targets = {'object': None, 'box': None}
        self.vision_grasp_ready = False
        
        # --- TF Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ======================================================================
        # COMMAND PUBLISHERS (Matrix-Driven)
        # ======================================================================
        self.pub_nav_goal = self.create_publisher(PoseStamped, '/nav/goal_pose', 10)
        self.pub_arm_cmd = self.create_publisher(String, '/arm/command', 10)
        self.pub_vision_mode = self.create_publisher(String, '/vision/mode', 10)
        self.pub_target_color = self.create_publisher(String, '/vision/active_color', 10)
        self.pub_arm_pose = self.create_publisher(PoseStamped, '/arm/target_pose', 10)
        
        # ======================================================================
        # FEEDBACK SUBSCRIBERS
        # ======================================================================
        self.sub_vision_pose = self.create_subscription(PoseStamped, '/vision/target_pose', self.vision_pose_callback, 10)
        self.sub_grasp_status = self.create_subscription(Bool, '/vision/grasp_status', self.grasp_status_callback, 10)
        self.sub_move_fb = self.create_subscription(Bool, '/nav/goal_reached', self.move_feedback_callback, 10)
        self.sub_manip_fb = self.create_subscription(Bool, '/arm/action_success', self.manip_feedback_callback, 10)
        self.sub_error = self.create_subscription(ErrorStatus, '/system/error_status', self.error_callback, 10)
        
        # ======================================================================
        # OUTPUT CONTROL MATRIX
        # ======================================================================
        self.control_matrix = {
            STATE_INIT:           {"chassis": ChassisCmd.IDLE,     "arm": ArmTask.RESET,     "vision": VisionMode.STANDBY,    "frame": RefFrame.NONE,   "target": TargetType.NONE},
            STATE_SEARCH:         {"chassis": ChassisCmd.EXPLORE,  "arm": ArmTask.STOW,      "vision": VisionMode.SCAN_ALL,   "frame": RefFrame.MAP,    "target": TargetType.ANY},
            STATE_MOVE_TO_OBJECT: {"chassis": ChassisCmd.NAVIGATE, "arm": ArmTask.STOW,      "vision": VisionMode.LOCK_COLOR, "frame": RefFrame.MAP,    "target": TargetType.OBJECT},
            STATE_GRASP:          {"chassis": ChassisCmd.BRAKE,    "arm": ArmTask.PICK_ACT,  "vision": VisionMode.LOCK_COLOR, "frame": RefFrame.CAMERA, "target": TargetType.OBJECT},
            STATE_MOVE_TO_BOX:    {"chassis": ChassisCmd.NAVIGATE, "arm": ArmTask.HOLD_OBJ,  "vision": VisionMode.LOCK_COLOR, "frame": RefFrame.MAP,    "target": TargetType.BOX},
            STATE_DROP:           {"chassis": ChassisCmd.BRAKE,    "arm": ArmTask.PLACE_ACT, "vision": VisionMode.LOCK_COLOR, "frame": RefFrame.CAMERA, "target": TargetType.BOX}
        }
        
        # ======================================================================
        # TIMERS
        # ======================================================================
        self.init_timer = self.create_timer(1.0, self.check_hardware_readiness)
        self.control_loop_timer = self.create_timer(0.1, self.execute_control_loop)
        self.get_logger().info('Master Control Node Initialized. Entering INIT state.')

    # ==========================================================================
    # MATRIX COMMAND DISPATCHER 
    # ==========================================================================
    def dispatch_matrix_commands(self):
        """Looks up the current state in the Control Matrix and publishes commands."""
        config = self.control_matrix[self.current_state]

        msg_arm = String()
        msg_arm.data = config['arm'].value
        self.pub_arm_cmd.publish(msg_arm)
        
        msg_vision = String()
        msg_vision.data = config['vision'].value
        self.pub_vision_mode.publish(msg_vision)
        
        msg_color = String()
        msg_color.data = self.active_target_color
        self.pub_target_color.publish(msg_color)

    def transition_to_state(self, new_state, log_msg=""):
        """Helper to change state and immediately dispatch matrix commands."""
        self.current_state = new_state
        if log_msg:
            self.get_logger().info(f"--- STATE: {new_state} | {log_msg} ---")
        self.dispatch_matrix_commands()

    # ==========================================================================
    # SYSTEM INITIALIZATION & HARDWARE CHECK
    # ==========================================================================
    def check_hardware_readiness(self):
        if self.current_state != STATE_INIT:
            self.init_timer.cancel()
            return
            
        if not self.hardware_ready['camera'] and self.count_publishers('/vision/target_pose') > 0:
            self.hardware_ready['camera'] = True
            self.get_logger().info('[Check] Camera Node: ONLINE')
            
        if not self.hardware_ready['nav'] and self.count_publishers('/nav/goal_reached') > 0:
            self.hardware_ready['nav'] = True
            self.get_logger().info('[Check] Navigation Node: ONLINE')
            
        if not self.hardware_ready['arm'] and self.count_publishers('/arm/action_success') > 0:
            self.hardware_ready['arm'] = True
            self.get_logger().info('[Check] Manipulator Node: ONLINE')
            
        if all(self.hardware_ready.values()):
            self.get_logger().info('>>> ALL SYSTEMS GO! Starting Mission... <<<')
            self.init_timer.cancel() 
            self.transition_to_state(STATE_SEARCH, "Commencing Global Patrol.")

    def error_callback(self, msg):
        if msg.error_state:
            self.get_logger().fatal(f'CRITICAL ERROR! Code: {msg.error_number}')
            raise SystemExit

    # ==========================================================================
    # VISION LOGIC & REAL-TIME TRACKING
    # ==========================================================================
    def vision_pose_callback(self, msg):
        """
        Dual-mode vision processor:
        1. SEARCH STATE: Populates Registry and checks for pairs.
        2. MOVE/ACTION STATES: Streams real-time 'camera_link' coordinates to the Arm.
        """
        try:
            color, item_type = msg.header.frame_id.split('_') 
        except ValueError:
            return 
            
        # ----------------------------------------------------------------------
        # MODE 1: GLOBAL SEARCH & REGISTRATION
        # ----------------------------------------------------------------------
        if self.current_state == STATE_SEARCH:
            if color not in self.color_registry:
                self.color_registry[color] = {'object': None, 'box': None}

            if item_type in ['object', 'box']:
                self.color_registry[color][item_type] = msg
                self.get_logger().debug(f"Registry Updated: [{color.upper()}] -> [{item_type.upper()}]")
                
            self.evaluate_registry_for_pairs()
            return
            
        # ----------------------------------------------------------------------
        # MODE 2: REAL-TIME TRACKING (For Arm Kinematics)
        # ----------------------------------------------------------------------
        if color != self.active_target_color:
            return
            
        if self.current_state in [STATE_MOVE_TO_OBJECT, STATE_GRASP]:
            if item_type == 'object':
                self.pub_arm_pose.publish(msg)
                
        elif self.current_state in [STATE_MOVE_TO_BOX, STATE_DROP]:
            if item_type == 'box':
                self.pub_arm_pose.publish(msg)

    def evaluate_registry_for_pairs(self):
        """Iterates through the Registry to find a complete target pair."""
        for color, items in self.color_registry.items():
            if items['object'] is not None and items['box'] is not None:
                self.get_logger().info(f"MATCH FOUND for [{color.upper()}]. Locking targets...")
                try:
                    t_obj = self.tf_buffer.lookup_transform(RefFrame.MAP.value, items['object'].header.frame_id, rclpy.time.Time())
                    map_obj_pose = tf2_geometry_msgs.do_transform_pose(items['object'], t_obj)
                    
                    t_box = self.tf_buffer.lookup_transform(RefFrame.MAP.value, items['box'].header.frame_id, rclpy.time.Time())
                    map_box_pose = tf2_geometry_msgs.do_transform_pose(items['box'], t_box)
                    
                    self.active_target_color = color
                    self.locked_targets['object'] = map_obj_pose
                    self.locked_targets['box'] = map_box_pose
                    
                    self.pub_nav_goal.publish(self.locked_targets['object'])
                    self.transition_to_state(STATE_MOVE_TO_OBJECT, f"Navigating to {color.upper()} OBJECT.")
                    return 
                    
                except TransformException as ex:
                    self.get_logger().error(f'TF Transform failed for {color}: {ex}')

    def grasp_status_callback(self, msg):
        """Optional: Triggers final action if vision confirms alignment."""
        self.vision_grasp_ready = msg.data
        if self.current_state == STATE_GRASP and self.vision_grasp_ready:
            self.get_logger().info('Vision confirmed alignment -> Re-dispatching PICK_ACT.')
            self.dispatch_matrix_commands()

    # ==========================================================================
    # HARDWARE FEEDBACK & STATE TRANSITIONS
    # ==========================================================================
    def move_feedback_callback(self, msg):
        """Handles Navigation Goal Reached events."""
        if not msg.data:
            return
            
        if self.current_state == STATE_MOVE_TO_OBJECT:
            self.transition_to_state(STATE_GRASP, "Arrived at Object. Awaiting precise pick.")
            
        elif self.current_state == STATE_MOVE_TO_BOX:
            self.transition_to_state(STATE_DROP, "Arrived at Box. Executing precise place.")

    def manip_feedback_callback(self, msg):
        """Handles Manipulator Action Success events."""
        if not msg.data:
            return
            
        if self.current_state == STATE_GRASP:
            self.pub_nav_goal.publish(self.locked_targets['box'])
            self.transition_to_state(STATE_MOVE_TO_BOX, f"Navigating to {self.active_target_color.upper()} BOX.")
            
        elif self.current_state == STATE_DROP:
            self.cycle_count += 1
            completed_color = self.active_target_color
            self.get_logger().info(f'[{completed_color.upper()}] Sorted! Cycle: {self.cycle_count}/{self.max_cycles}')
            
            if self.cycle_count >= self.max_cycles:
                self.get_logger().info('MISSION ACCOMPLISHED. Shutting down.')
                raise SystemExit 
            else:
                if completed_color in self.color_registry:
                    del self.color_registry[completed_color]
                    
                self.active_target_color = "NONE"
                self.locked_targets = {'object': None, 'box': None}
                self.vision_grasp_ready = False
                
                self.transition_to_state(STATE_SEARCH, "Restarting global patrol for next pair.")

    # ==========================================================================
    # MAIN CONTROL LOOP
    # ==========================================================================
    def execute_control_loop(self):
        """
        10Hz Timer Callback.
        Use this for watchdog, periodic heartbeat, or continuous TF tracking.
        Currently handled event-driven via transition_to_state().
        """
        pass 


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()