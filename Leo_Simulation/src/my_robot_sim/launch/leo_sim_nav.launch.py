import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command 

def generate_launch_description():
    # Get the paths of two different packages
    sim_pkg_dir = get_package_share_directory('my_robot_sim')
    leo_pkg_dir = get_package_share_directory('leo_description') # Get the package path for the Leo rover

    # Configuration files and model paths
    urdf_file = os.path.join(leo_pkg_dir, 'urdf', 'leo_sim.urdf.xacro') 
    world_file = os.path.join(sim_pkg_dir, 'worlds', 'simple_room.sdf')  
    bridge_config = os.path.join(sim_pkg_dir, 'config', 'bridge.yaml')
    slam_params_file = os.path.join(sim_pkg_dir, 'config', 'mapper_params_online_async.yaml')
    nav2_params_file = os.path.join(sim_pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(sim_pkg_dir, 'rviz', 'sim.rviz')
    robot_desc = Command(['xacro ', urdf_file])

    # 1. Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 2. Launch Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 3. Launch Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Spawn the robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'simple_room','-topic', 'robot_description', '-name', 'leo_sim', '-z', '0.02'],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=8.0,  # Delay time (seconds), change to 8.0 or 10.0 if computer loads Gazebo slowly
        actions=[spawn_robot]
    )

    # Use the ros2 command line tool to send a tiny velocity command to wake up the odometry plugin
    kickstart_odom = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', 
            '-t','30',
            '-r','10', 
            '/cmd_vel', 
            'geometry_msgs/msg/Twist', 
            '{linear: {x: 0.001, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
        ],
        output='screen'
    )

    # Delayed execution (wait for the rover to spawn and load stably in Gazebo before sending the command)
    delayed_kickstart = TimerAction(
        period=12.0, 
        actions=[kickstart_odom]
    )

    # 5. ROS-GZ Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )

    # 6. Launch Nav2 (pure navigation mode)
    nav2_bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items()
    )

    # 7. RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # Use arguments to load the configuration file
        arguments=['-d', rviz_config_file],
        # Parameter settings
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 8. Launch SLAM Toolbox
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params_file
        }.items()
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        joint_state_publisher,
        bridge,
        delayed_spawn,
        delayed_kickstart,
        rviz2_node,
        slam_toolbox_node,
        nav2_bringup_node
    ])