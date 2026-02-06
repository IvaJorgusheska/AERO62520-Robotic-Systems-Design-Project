import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import json
import subprocess

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # 1. 初始化 Nav2 API
        self.navigator = BasicNavigator()
        
        # 2. 订阅外部指令话题
        # 假设指令格式为 JSON: {"mode": "nav", "x": 1.5, "y": 2.0} 或 {"mode": "explore"}
        self.subscription = self.create_subscription(
            String,
            '/mission_command',
            self.command_callback,
            10)
            
        # 3. 探索节点进程句柄 (用于开启/关闭探索)
        self.explore_process = None
        
        self.get_logger().info('Mission Controller Started. Waiting for commands...')

    def command_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
            mode = cmd.get('mode')
            
            if mode == 'explore':
                self.start_exploration()
            elif mode == 'nav':
                x = cmd.get('x', 0.0)
                y = cmd.get('y', 0.0)
                self.start_navigation(x, y)
            elif mode == 'stop':
                self.stop_all()
                
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON command')

    def start_exploration(self):
        self.get_logger().info('Switching to EXPLORATION mode...')
        
        # 1. 取消当前 Nav2 的所有导航任务
        self.navigator.cancelTask()
        
        # 2. 启动 explore_lite 节点 (如果还没运行)
        # 注意：更好的方式是使用 Lifecycle Node 管理，但 subprocess 最简单直接
        if self.explore_process is None:
            cmd = ['ros2', 'run', 'explore_lite', 'explore']
            # 可以在这里添加 params file 参数
            self.explore_process = subprocess.Popen(cmd)
            self.get_logger().info('Explore Lite launched.')
        else:
            self.get_logger().info('Explore Lite is already running.')

    def start_navigation(self, x, y):
        self.get_logger().info(f'Switching to NAVIGATION mode. Target: {x}, {y}')
        
        # 1. 杀死/停止探索节点，防止它抢夺控制权
        if self.explore_process is not None:
            self.explore_process.terminate()
            self.explore_process.wait()
            self.explore_process = None
            self.get_logger().info('Explore Lite stopped.')
        
        # 2. 发送目标点给 Nav2
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.orientation.w = 1.0 # 默认朝向，也可以由指令指定
        
        self.navigator.goToPose(goal_pose)
        
        # 注意：goToPose 是非阻塞的，你可以用 self.navigator.isTaskComplete() 检查状态

    def stop_all(self):
        self.navigator.cancelTask()
        if self.explore_process:
            self.explore_process.terminate()
            self.explore_process = None
        self.navigator.lifecycleShutdown()

def main():
    rclpy.init()
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()