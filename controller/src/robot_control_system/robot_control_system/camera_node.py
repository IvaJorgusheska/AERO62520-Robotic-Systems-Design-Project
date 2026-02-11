import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
##ADD Only then: add TF (camera_link → base_link) 

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # ----------------------------
        # Publishers
        # ----------------------------
        self.pub_detected = self.create_publisher(Bool, '/detected_object', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/object_pose', 10)

        # ----------------------------
        # Load YOLO model
        # ----------------------------
        self.model = YOLO("best.pt")
        self.get_logger().info("YOLO model loaded")

        # ----------------------------
        # RealSense setup
        # ----------------------------
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        self.align = rs.align(rs.stream.color)

        # Camera intrinsics
        profile = self.pipeline.get_active_profile()
        color_stream = profile.get_stream(rs.stream.color)
        intr = color_stream.as_video_stream_profile().get_intrinsics()

        self.fx = intr.fx
        self.fy = intr.fy
        self.cx = intr.ppx
        self.cy = intr.ppy

        self.get_logger().info("RealSense initialized")

        # ----------------------------
        # Timer (10 Hz)
        # ----------------------------
        self.timer = self.create_timer(0.1, self.main_loop)

    def main_loop(self):
        # ----------------------------
        # Get frames
        # ----------------------------
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())

        # ----------------------------
        # Run YOLO
        # ----------------------------
        results = self.model(color_image, conf=0.5, verbose=False)

        if len(results[0].boxes) == 0:
            msg = Bool()
            msg.data = False
            self.pub_detected.publish(msg)
            return

        # ----------------------------
        # Single-object logic (first box)
        # ----------------------------
        box = results[0].boxes[0]
        x1, y1, x2, y2 = map(int, box.xyxy[0])

        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)

        depth = depth_frame.get_distance(u, v)

        if depth <= 0.0:
            return

        # ----------------------------
        # Camera-frame 3D coordinates
        # ----------------------------
        X = (u - self.cx) / self.fx * depth
        Y = (v - self.cy) / self.fy * depth
        Z = depth

        # ----------------------------
        # Publish detection flag
        # ----------------------------
        detected_msg = Bool()
        detected_msg.data = True
        self.pub_detected.publish(detected_msg)

        # ----------------------------
        # Publish pose (camera frame)
        # ----------------------------
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'camera_link'

        pose.pose.position.x = float(X)
        pose.pose.position.y = float(Y)
        pose.pose.position.z = float(Z)
        pose.pose.orientation.w = 1.0

        self.pub_pose.publish(pose)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
