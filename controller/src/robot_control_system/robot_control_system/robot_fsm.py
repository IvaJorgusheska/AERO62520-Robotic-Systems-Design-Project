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
    - Logic: Send 'map' object pose to Nav2.
    - Transition: If Navigation returns 'Goal Reached' -> [3] GRASP.

[3] GRASP:
    - Logic: Chassis halts. Send 'camera_link' relative pose to Arm for precise pick.
    - Transition: If Vision returns False (Object picked) -> [4] MOVE_TO_BOX.

[4] MOVE_TO_BOX:
    - Logic: Send matching Box 'map' pose to Nav2.
    - Transition: If Navigation returns 'Goal Reached' -> [5] DROP.

[5] DROP:
    - Logic: Chassis halts. Send 'camera_link' relative pose to Arm for precise place.
    - Transition: 
        - If Total Cycles == 3 -> [Exit Mission / Shutdown].
        - Else -> Reset CURRENT_TARGET_COLOR to 'NONE' -> [1] SEARCH.

2. OUTPUT CONTROL MATRIX (Classic & Refined Version)
--------------------------------------------------------------------------------
| STATE        | CHASSIS_CMD | ARM_TASK  | VISION_MODE  | TF_FRAME    | TARGET_TYPE |
|--------------|-------------|-----------|--------------|-------------|-------------|
| [0] INIT     | IDLE        | RESET     | STANDBY      | NONE        | NONE        |
| [1] SEARCH   | EXPLORE     | STOW      | SCAN_ALL     | MAP         | ANY         |
| [2] MOVE_OBJ | NAVIGATE    | STOW      | LOCK_COLOR   | MAP         | OBJECT      |
| [3] GRASP    | BRAKE       | PICK_ACT  | LOCK_COLOR   | CAMERA_LINK | OBJECT      |
| [4] MOVE_BOX | NAVIGATE    | HOLD_OBJ  | LOCK_COLOR   | MAP         | BOX         |
| [5] DROP     | BRAKE       | PLACE_ACT | LOCK_COLOR   | CAMERA_LINK | BOX         |

================================================================================
"""

import random
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, PointStamped

# TF2 packages for coordinate transformations
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs 

# Custom interfaces
from my_robot_interfaces.msg import ObjectTarget, GripperState, GripperPose

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

class TfFrame(Enum):
    NONE        = "NONE"
    MAP         = "map"         
    CAMERA_LINK = "camera_link" 

class TargetType(Enum):
    NONE   = "none"
    ANY    = "any"
    OBJECT = "object"
    BOX    = "box"

# ==============================================================================
# 3. MAIN CONTROLLER NODE
# ==============================================================================
class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        
        self.current_state = STATE_INIT
        self.cycle_count = 0  
        self.max_cycles = 3   
        
        self.hardware_ready = {'camera': False, 'nav': False, 'arm': False}
        self.map_hash = {}    
        self.camera_hash = {} 
        self.active_target_color = "NONE" 
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ----------------------------------------------------------------------
        # Publishers 
        # ----------------------------------------------------------------------
        self.pub_nav_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_arm_pose = self.create_publisher(GripperPose, '/arm/grasp_pose', 10)
        self.pub_arm_grip = self.create_publisher(GripperState, '/arm/grasp_status', 10)
        self.pub_target_color = self.create_publisher(String, '/vision/active_color', 10)
        
        # ----------------------------------------------------------------------
        # Subscribers
        # ----------------------------------------------------------------------
        self.sub_vision_target = self.create_subscription(ObjectTarget, '/detected_object', self.vision_target_callback, 10)
        self.sub_vision_state = self.create_subscription(Bool, '/detection_state', self.vision_state_callback, 10)
        self.sub_move_fb = self.create_subscription(Bool, '/nav/goal_reached', self.move_feedback_callback, 10)
        
        # ----------------------------------------------------------------------
        # Output Control Matrix
        # ----------------------------------------------------------------------
        self.control_matrix = {
            STATE_INIT:           {"chassis": ChassisCmd.IDLE,     "arm": ArmTask.RESET,     "frame": TfFrame.NONE,        "target": TargetType.NONE},
            STATE_SEARCH:         {"chassis": ChassisCmd.EXPLORE,  "arm": ArmTask.STOW,      "frame": TfFrame.MAP,         "target": TargetType.ANY},
            STATE_MOVE_TO_OBJECT: {"chassis": ChassisCmd.NAVIGATE, "arm": ArmTask.STOW,      "frame": TfFrame.MAP,         "target": TargetType.OBJECT},
            STATE_GRASP:          {"chassis": ChassisCmd.BRAKE,    "arm": ArmTask.PICK_ACT,  "frame": TfFrame.CAMERA_LINK, "target": TargetType.OBJECT},
            STATE_MOVE_TO_BOX:    {"chassis": ChassisCmd.NAVIGATE, "arm": ArmTask.HOLD_OBJ,  "frame": TfFrame.MAP,         "target": TargetType.BOX},
            STATE_DROP:           {"chassis": ChassisCmd.BRAKE,    "arm": ArmTask.PLACE_ACT, "frame": TfFrame.CAMERA_LINK, "target": TargetType.BOX}
        }
        
        # ----------------------------------------------------------------------
        # Timers
        # ----------------------------------------------------------------------
        self.init_timer = self.create_timer(1.0, self.check_hardware_readiness)
        self.control_loop_timer = self.create_timer(0.1, self.execute_control_loop) 
        self.mission_timer = self.create_timer(1200.0, lambda: [self.get_logger().fatal("Timeout!"), exit()])
        
        self.get_logger().info('Master Control Node Initialized. Entering INIT state.')

    # ==========================================================================
    # CORE: TIMER-DRIVEN CONDITION CHECKER (10Hz)
    # ==========================================================================
    def execute_control_loop(self):
        """Periodically polls the registry to check if state transition conditions are met."""
        if self.current_state == STATE_SEARCH:
            found_color = self.check_for_target_pair()
            if found_color != "NONE":
                self.active_target_color = found_color
                self.transition_to_state(STATE_MOVE_TO_OBJECT, f"Pair found: {found_color.upper()}")

    def check_for_target_pair(self) -> str:
        """Scans HashMap for a complete Object-Box pair."""
        for color, items in self.map_hash.items():
            if items['object'] is not None and items['box'] is not None:
                return color
        return "NONE"

    # ==========================================================================
    # CORE: STATE TRANSITION & MATRIX DISPATCHER
    # ==========================================================================
    def transition_to_state(self, new_state, log_msg=""):
        self.current_state = new_state
        if log_msg:
            self.get_logger().info(f"--- STATE: {new_state} | {log_msg} ---")
        self.dispatch_matrix_commands()

    def dispatch_matrix_commands(self):
        """Dispatches commands to Chassis, Arm, and other nodes strictly based on the matrix."""
        config = self.control_matrix[self.current_state]
        chassis_cmd, arm_task, tf_frame, target_type = config["chassis"], config["arm"], config["frame"], config["target"].value

        # Always publish active target color
        msg_color = String()
        msg_color.data = self.active_target_color
        self.pub_target_color.publish(msg_color)

        # Parse Chassis Actions
        if chassis_cmd == ChassisCmd.EXPLORE:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = random.uniform(-2.5, 2.5)
            pose.pose.position.y = random.uniform(-2.5, 2.5)
            pose.pose.orientation.w = 1.0 
            self.pub_nav_goal.publish(pose)
            
        elif chassis_cmd == ChassisCmd.NAVIGATE:
            if self.active_target_color in self.map_hash and tf_frame == TfFrame.MAP:
                target_pt = self.map_hash[self.active_target_color][target_type]
                if target_pt is not None: 
                    pose = PoseStamped()
                    pose.header = target_pt.header
                    pose.pose.position = target_pt.point
                    pose.pose.orientation.w = 1.0
                    self.pub_nav_goal.publish(pose)

        # Parse Arm Actions (Translation from Macro Task to real coordinates)
        if arm_task in [ArmTask.PICK_ACT, ArmTask.PLACE_ACT]:
            if self.active_target_color in self.camera_hash and tf_frame == TfFrame.CAMERA_LINK:
                cam_pt = self.camera_hash[self.active_target_color][target_type]
                if cam_pt is not None: 
                    pose_msg = GripperPose()
                    pose_msg.x, pose_msg.y, pose_msg.z = float(cam_pt.x), float(cam_pt.y), float(cam_pt.z)
                    pose_msg.roll, pose_msg.pitch, pose_msg.yaw = 180.0, 0.0, 0.0
                    self.pub_arm_pose.publish(pose_msg)

        # Parse Gripper State
        state_msg = GripperState()
        state_msg.grip = (arm_task in [ArmTask.PICK_ACT, ArmTask.HOLD_OBJ])
        self.pub_arm_grip.publish(state_msg)

    # ==========================================================================
    # AUXILIARY: HARDWARE CHECK
    # ==========================================================================
    def check_hardware_readiness(self):
        """Probes the ROS 2 graph to ensure actuators and sensors are online."""
        if self.current_state != STATE_INIT:
            self.init_timer.cancel()
            return
            
        self.hardware_ready['camera'] = (self.count_publishers('/detection_state') > 0) or self.hardware_ready['camera']
        self.hardware_ready['nav'] = (self.count_subscribers('/goal_pose') > 0) or self.hardware_ready['nav']
        self.hardware_ready['arm'] = (self.count_subscribers('/arm/grasp_pose') > 0) or self.hardware_ready['arm']
            
        if all(self.hardware_ready.values()):
            self.get_logger().info('>>> ALL SYSTEMS GO! Hardware verified. <<<')
            self.init_timer.cancel() 
            self.transition_to_state(STATE_SEARCH, "Commencing Global Patrol.")

    # ==========================================================================
    # LOGIC: VISION DATA PUMP & TF
    # ==========================================================================
    def vision_target_callback(self, msg: ObjectTarget):
        """Pure data pipeline: stores local camera coordinates and maps them to global."""
        color, item_type = msg.color, msg.name 
        if color == "unknown": return
            
        if color not in self.camera_hash:
            self.camera_hash[color] = {'object': None, 'box': None}
        self.camera_hash[color][item_type] = msg 
        
        try:
            pt = PointStamped()
            pt.header.frame_id = 'camera_link'
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point.x, pt.point.y, pt.point.z = msg.x, msg.y, msg.z
            
            t = self.tf_buffer.lookup_transform('map', 'camera_link', rclpy.time.Time())
            map_pt = tf2_geometry_msgs.do_transform_point(pt, t)
            
            if color not in self.map_hash:
                self.map_hash[color] = {'object': None, 'box': None}
            self.map_hash[color][item_type] = map_pt
            
        except TransformException:
            pass

    # ==========================================================================
    # LOGIC: SIGNAL DRIVEN TRANSITIONS
    # ==========================================================================
    def move_feedback_callback(self, msg: Bool):
        """Triggers the next sequence when the chassis reaches its destination."""
        if not msg.data: return
        
        if self.current_state == STATE_SEARCH:
            self.transition_to_state(STATE_SEARCH, "Continuing map exploration...")
        elif self.current_state == STATE_MOVE_TO_OBJECT:
            self.transition_to_state(STATE_GRASP, "Arrived at Object. Awaiting Arm Action.")
        elif self.current_state == STATE_MOVE_TO_BOX:
            self.transition_to_state(STATE_DROP, "Arrived at Box. Awaiting Arm Action.")

    def vision_state_callback(self, msg: Bool):
        """
        Relies on vision occlusion/disappearance to confirm blind-grasping success.
        msg.data == False means the camera sees nothing (object is picked or dropped in box).
        """
        if not msg.data: 
            if self.current_state == STATE_GRASP:
                self.transition_to_state(STATE_MOVE_TO_BOX, "Vision confirmed: Object picked up!")
            elif self.current_state == STATE_DROP:
                self.cycle_count += 1
                completed_color = self.active_target_color
                
                if self.cycle_count >= self.max_cycles:
                    self.get_logger().info('ALL TASKS COMPLETED. Shutting down.')
                    raise SystemExit 
                else:
                    if completed_color in self.map_hash:
                        del self.map_hash[completed_color]
                    if completed_color in self.camera_hash:
                        del self.camera_hash[completed_color]
                        
                    self.active_target_color = "NONE"
                    self.transition_to_state(STATE_SEARCH, f"Cycle {self.cycle_count} done! Restarting patrol.")

def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(RobotControlNode())
    except SystemExit:
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()