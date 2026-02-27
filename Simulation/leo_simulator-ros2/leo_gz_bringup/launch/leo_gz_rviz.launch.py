# Copyright 2023 Fictionlab sp. z o.o.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Setup project paths
    pkg_project_gazebo = get_package_share_directory("leo_gz_bringup")
    pkg_project_worlds = get_package_share_directory("leo_gz_worlds")

    # Launch arguments
    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value=os.path.join(pkg_project_worlds, "worlds", "leo_empty.sdf"),
        description="Path to the Gazebo world file",
    )

    robot_ns = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace",
    )

    x_pos = DeclareLaunchArgument(
        "x",
        default_value="0.5",
        description="Initial X position of the robot (meters)",
    )

    y_pos = DeclareLaunchArgument(
        "y",
        default_value="0.0",
        description="Initial Y position of the robot (meters)",
    )

    z_pos = DeclareLaunchArgument(
        "z",
        default_value="1.65",
        description="Initial Z position of the robot (meters)",
    )

    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to launch RViz",
    )

    rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(
            pkg_project_gazebo, "rviz", "leo_gz.rviz"
        ),
        description="Path to RViz configuration file",
    )

    # Include the existing leo_gz launch file to start Gazebo
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_gazebo, "launch", "leo_gz.launch.py")
        ),
        launch_arguments={
            "sim_world": LaunchConfiguration("sim_world"),
            "robot_ns": LaunchConfiguration("robot_ns"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
        }.items(),
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("use_rviz"), "' == 'true'"])
        ),
        output="screen",
    )

    return LaunchDescription(
        [
            sim_world,
            robot_ns,
            x_pos,
            y_pos,
            z_pos,
            use_rviz,
            rviz_config,
            gz_sim_launch,
            rviz_node,
        ]
    )
