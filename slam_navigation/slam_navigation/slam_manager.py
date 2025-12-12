#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import PointCloud2, LaserScan, Image
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
import tf2_geometry_msgs
import threading
import time


class SlamManager(Node):
    def __init__(self):
        super().__init__('slam_manager')

        # 参数
        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('fusion_method', 'weighted_average')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')

        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.fusion_method = self.get_parameter('fusion_method').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)


        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        self.slam_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/slam_toolbox/pose',
            self.slam_pose_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            scan_qos
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos
        )


        self.fused_pose_pub = self.create_publisher(
            PoseStamped,
            '/fused_pose',
            10
        )

        self.fused_map_pub = self.create_publisher(
            OccupancyGrid,
            '/fused_map',
            map_qos
        )

        self.status_pub = self.create_publisher(
            MarkerArray,
            '/slam_status',
            10
        )


        self.current_pose = None
        self.odom_pose = None
        self.slam_pose = None
        self.last_scan = None
        self.map_data = None


        self.pose_lock = threading.Lock()
        self.map_lock = threading.Lock()


        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_fused_data)
        self.status_timer = self.create_timer(0.5, self.publish_status)


    def slam_pose_callback(self, msg):

        with self.pose_lock:
            self.slam_pose = msg.pose.pose


            if self.odom_pose is not None:
                self.current_pose = self.fuse_poses(self.odom_pose, self.slam_pose)
            else:
                self.current_pose = self.slam_pose

    def odom_callback(self, msg):

        with self.pose_lock:
            self.odom_pose = msg.pose.pose


            if self.slam_pose is None:
                self.current_pose = self.odom_pose

    def scan_callback(self, msg):

        self.last_scan = msg

    def fuse_poses(self, odom_pose, slam_pose):

        if self.fusion_method == 'weighted_average':
            # 加权平均融合
            weight_slam = 0.7
            weight_odom = 0.3

            fused_pose = odom_pose


            fused_pose.position.x = (weight_slam * slam_pose.position.x +
                                     weight_odom * odom_pose.position.x)
            fused_pose.position.y = (weight_slam * slam_pose.position.y +
                                     weight_odom * odom_pose.position.y)
            fused_pose.position.z = (weight_slam * slam_pose.position.z +
                                     weight_odom * odom_pose.position.z)


            fused_pose.orientation.x = (weight_slam * slam_pose.orientation.x +
                                        weight_odom * odom_pose.orientation.x)
            fused_pose.orientation.y = (weight_slam * slam_pose.orientation.y +
                                        weight_odom * odom_pose.orientation.y)
            fused_pose.orientation.z = (weight_slam * slam_pose.orientation.z +
                                        weight_odom * odom_pose.orientation.z)
            fused_pose.orientation.w = (weight_slam * slam_pose.orientation.w +
                                        weight_odom * odom_pose.orientation.w)

            return fused_pose

        elif self.fusion_method == 'slam_priority':

            return slam_pose

        else:

            return slam_pose

    def publish_fused_data(self):

        if self.current_pose is None:
            return


        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.global_frame
        pose_msg.pose = self.current_pose
        self.fused_pose_pub.publish(pose_msg)


        transform_msg = TransformStamped()
        transform_msg.header.stamp = pose_msg.header.stamp
        transform_msg.header.frame_id = self.global_frame
        transform_msg.child_frame_id = self.robot_frame
        transform_msg.transform.translation.x = self.current_pose.position.x
        transform_msg.transform.translation.y = self.current_pose.position.y
        transform_msg.transform.translation.z = self.current_pose.position.z
        transform_msg.transform.rotation = self.current_pose.orientation

        self.tf_broadcaster.sendTransform(transform_msg)

    def publish_status(self):

        markers = MarkerArray()


        status_marker = Marker()
        status_marker.header.frame_id = self.robot_frame
        status_marker.header.stamp = self.get_clock().now().to_msg()
        status_marker.ns = "slam_status"
        status_marker.id = 0
        status_marker.type = Marker.TEXT_VIEW_FACING
        status_marker.action = Marker.ADD
        status_marker.pose.position.x = 0.0
        status_marker.pose.position.y = 0.0
        status_marker.pose.position.z = 1.0
        status_marker.scale.z = 0.1


        if self.current_pose is not None:
            status_text = "SLAM: ACTIVE\n"
            status_text += f"Pos: ({self.current_pose.position.x:.2f}, "
            status_text += f"{self.current_pose.position.y:.2f})\n"
            status_text += f"Fusion: {self.fusion_method}"
            status_marker.color.a = 1.0
            status_marker.color.g = 1.0
        else:
            status_text = "SLAM: INITIALIZING"
            status_marker.color.a = 1.0
            status_marker.color.r = 1.0
            status_marker.color.g = 1.0
            status_marker.color.b = 0.0

        status_marker.text = status_text
        markers.markers.append(status_marker)

        self.status_pub.publish(markers)

    def fuse_maps(self, slam_map, nvblox_map):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SlamManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()