
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_nvblox = LaunchConfiguration('use_nvblox', default='true')
    use_slam_toolbox = LaunchConfiguration('use_slam_toolbox', default='true')
    use_nav2 = LaunchConfiguration('use_nav2', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nvblox_params_file = LaunchConfiguration('nvblox_params_file')


    slam_navigation_dir = get_package_share_directory('slam_navigation')


    declare_use_nvblox = DeclareLaunchArgument(
        'use_nvblox',
        default_value='true',
        description='Whether to use nvblox for 3D reconstruction'
    )

    declare_use_slam_toolbox = DeclareLaunchArgument(
        'use_slam_toolbox',
        default_value='true',
        description='Whether to use slam_toolbox for 2D SLAM'
    )

    declare_use_nav2 = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Whether to use Nav2 for navigation'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            slam_navigation_dir, 'config', 'slam_toolbox_params.yaml'),
        description='Full path to the SLAM parameters file'
    )

    declare_nvblox_params_file = DeclareLaunchArgument(
        'nvblox_params_file',
        default_value=os.path.join(
            slam_navigation_dir, 'config', 'nvblox_params.yaml'),
        description='Full path to the nvblox parameters file'
    )
    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}]
    )


    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(use_slam_toolbox)
    )


    cmd_vel_transformer_node = Node(
        package='slam_navigation',
        executable='cmd_vel_transformer',
        name='cmd_vel_transformer',
        output='screen',
        parameters=[{
            'wheel_base': 0.5,
            'track_width': 0.4,
            'max_linear_vel': 1.0,
            'max_angular_vel': 1.0,
            'use_sim_time': use_sim_time
        }]
    )


    slam_manager_node = Node(
        package='slam_navigation',
        executable='slam_manager',
        name='slam_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'fusion_method': 'weighted_average',
            'publish_rate': 10.0
        }]
    )

    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nvblox_ros'),
                'launch',
                'nvblox.launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': nvblox_params_file,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_nvblox)
    )


    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': os.path.join(slam_navigation_dir, 'params', 'nav2_params.yaml'),
            'use_sim_time': use_sim_time,
            'autostart': 'true'
        }.items(),
        condition=IfCondition(use_nav2)
    )

    return LaunchDescription([
        declare_use_nvblox,
        declare_use_slam_toolbox,
        declare_use_nav2,
        declare_use_sim_time,
        declare_slam_params_file,
        declare_nvblox_params_file,

        tf_publisher,
        slam_toolbox_node,
        cmd_vel_transformer_node,
        slam_manager_node,
        nvblox_launch,
        nav2_launch,
    ])