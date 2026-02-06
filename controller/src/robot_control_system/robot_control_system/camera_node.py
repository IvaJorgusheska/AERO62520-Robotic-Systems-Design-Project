import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs  # Essential for do_transform_pose
from my_robot_interfaces.msg import ObjectTarget

# === 2. Define Object Class ===
class DetectedObject:
    """
    Class to store object attributes.
    """
    def __init__(self, name, color, x, y, z, qx, qy, qz, qw):
        self.name = name
        self.color = color
        self.position = {'x': x, 'y': y, 'z': z}
        self.orientation = {'x': qx, 'y': qy, 'z': qz, 'w': qw}

class CameraControlNode(Node):
    def __init__(self):
        super().__init__('camera_control_node')

        # === 1. Internal State Indicator ===
        self.mark = False
        self.last_mark = False # To detect state changes (Edge detection)
        self.current_object = None # Instance of DetectedObject

        # Publishers
        self.pub_detected = self.create_publisher(Bool, '/detected_object', 10)
        self.pub_detail = self.create_publisher(ObjectTarget, '/object_detail', 10)

        # TF Tools
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer (10Hz)
        self.timer = self.create_timer(0.1, self.main_loop)
        
        self.get_logger().info('Camera Node Initialized.')

    # === 3. Coordinate Transform Function ===
    def get_coordinate_in_base_link(self, obj_local):
        """
        Input: Object in /camera_link frame
        Output: PoseStamped in /base_link frame
        """
        try:
            # 1. Create a PoseStamped object from raw data
            p_cam = PoseStamped()
            p_cam.header.frame_id = "camera_link" # Assuming camera frame name
            p_cam.header.stamp = rclpy.time.Time().to_msg()
            
            p_cam.pose.position.x = obj_local.position['x']
            p_cam.pose.position.y = obj_local.position['y']
            p_cam.pose.position.z = obj_local.position['z']
            p_cam.pose.orientation.x = obj_local.orientation['x']
            p_cam.pose.orientation.y = obj_local.orientation['y']
            p_cam.pose.orientation.z = obj_local.orientation['z']
            p_cam.pose.orientation.w = obj_local.orientation['w']

            # 2. Look up transform from base_link to camera_link
            # Note: lookup_transform(target_frame, source_frame)
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                'camera_link', 
                rclpy.time.Time())

            # 3. Perform the calculation
            p_base = tf2_geometry_msgs.do_transform_pose(p_cam, transform)
            
            return p_base

        except Exception as e:
            self.get_logger().warning(f'TF Transform failed: {str(e)}')
            return None

    # === 4. Camera Function (Simulation) ===
    def camera_logic(self):
        """
        Simulates camera detection logic.
        Pseudocode logic included in comments.
        """
        # --- Pseudocode Start ---
        # image = camera.get_image()
        # targets = model.detect(image)
        # is_target_found = len(targets) > 0
        
        # Simulation: Let's pretend we found an object named "Apple"
        # Toggle this manually or add random logic to test
        is_target_found = True 
        
        if is_target_found:
            self.mark = True
            
            # Logic: If mark changed from False to True (Rising Edge)
            if self.last_mark == False:
                # Send True to /detected_object
                msg = Bool()
                msg.data = True
                self.pub_detected.publish(msg)
                self.get_logger().info('Target Detected! Published to /detected_object')

            # Create Object class instance (Simulated coordinates relative to camera)
            # x=1.0m ahead, no rotation
            self.current_object = DetectedObject("Apple", "Red", 1.0, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0)
        else:
            self.mark = False
            self.current_object = None

        # Update last_mark for next iteration
        self.last_mark = self.mark
        # --- Pseudocode End ---

    # === 5. Main Loop (10Hz) ===
    def main_loop(self):
        # 1. Call camera function
        self.camera_logic()

        # 2. Publish object details
        msg_detail = ObjectTarget()
        msg_detail.observe_state = self.mark
        
        if self.mark is False:
            msg_detail.object_name = "None"
        else:
            # Safe check if object exists
            if self.current_object:
                msg_detail.object_name = self.current_object.name
                msg_detail.color = self.current_object.color
                
                # 3. Coordinate Transformation
                # Transform from camera frame to base_link frame
                transformed_pose = self.get_coordinate_in_base_link(self.current_object)

                # 4. Publish Standard TF (if transform succeeded)
                if transformed_pose:
                    t = TransformStamped()
                    
                    # Fill Header
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = "base_link"       # Parent Frame
                    t.child_frame_id = self.current_object.name # Child Frame (e.g. "Apple")

                    # Fill Transform from the calculated pose
                    t.transform.translation.x = transformed_pose.pose.position.x
                    t.transform.translation.y = transformed_pose.pose.position.y
                    t.transform.translation.z = transformed_pose.pose.position.z
                    t.transform.rotation = transformed_pose.pose.orientation

                    # Broadcast
                    self.tf_broadcaster.sendTransform(t)

        # Publish the detail message
        self.pub_detail.publish(msg_detail)

def main(args=None):
    rclpy.init(args=args)
    node = CameraControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()