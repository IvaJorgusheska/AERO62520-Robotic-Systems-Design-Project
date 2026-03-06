
# 🤖 Leo Rover Autonomous Navigation Simulation

This project is a full-stack mobile robot simulation workspace based on **ROS 2 Jazzy** and **Gazebo Harmonic**. The core model is based on the Leo Rover (a four-wheel skid-steering chassis), deeply integrating SLAM Toolbox (real-time mapping), Nav2 (autonomous navigation), and a state-machine-based navigation control microservice, achieving closed-loop control for autonomous exploration, target grasping, and placement in indoor environments.

## 🛠️ Dependencies

This project was developed and tested in the following environment:
* **OS:** Ubuntu 24.04 LTS
* **ROS 2:** Jazzy Jalisco
* **Gazebo:** Harmonic (Ignition v8)

Please ensure the system has the following official ROS 2 core components installed:
```bash
sudo apt update
sudo apt install ros-jazzy-xacro ros-jazzy-ros-gz
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-teleop-twist-keyboard


```

## 📁 Workspace Structure

```text
Leo_Simulation/
├── README.md
└── src/
    ├── leo_description/        # Robot physical and visual description package
    │   ├── models/             # 3D Mesh model files
    │   └── urdf/
    │       ├── macros.xacro    # Core macro definitions (includes high-freq lidar and chassis virtual wheelbase compensation)
    │       └── leo_sim.urdf.xacro # Main simulation entry model
    │
    └── my_robot_sim/           # Simulation control, navigation, and launch package
        ├── launch/
        │   └── leo_sim_nav.launch.py # One-click launch file (includes Gazebo, RViz, SLAM, Nav2)
        ├── worlds/
        │   └── simple_room.sdf # Custom 4x4m indoor environment
        ├── config/
        │   ├── nav2_params.yaml             # Nav2 parameters (RPP curve optimization/AEB configuration)
        │   └── mapper_params_online_async.yaml # SLAM optimization parameters
        


```

## 🚀 Quick Start

### 1. Build the Workspace

```bash
cd ~/Leo_Simulation
colcon build --symlink-install
source install/setup.bash


```

### 2. Launch the Core Co-Simulation

Pull up the physical environment, robot, mapping, and navigation stack with one click:

```bash
ros2 launch my_robot_sim leo_sim_nav.launch.py


```

*💡 Note: The Launch file embeds an auto-wake-up mechanism (injecting a tiny velocity of 1mm/s 10 seconds after startup), perfectly solving the map deadlock issue caused by the Gazebo differential drive plugin not publishing Odom when stationary.*

### 3. Launch the Navigation Execution Engine (Microservice)

Run the non-blocking Action Client node you wrote in a new terminal:

```bash
source install/setup.bash
ros2 run robot_control_system nav_controller_node


```

## 🎮 Navigation Microservice API Interface Description

`nav_controller_node` is an independent spatial execution engine that entirely receives instructions from an external state machine (the brain) via ROS 2 Topics and feeds back progress. You can debug the interface in the terminal using the following commands:

* **1. Start Random Exploration (EXPLORING)**

```bash
ros2 topic pub -1 /status/moving std_msgs/msg/Bool "{data: true}"


```

* **2. Target Object Found, Issue Grasping Coordinates (APPROACHING_OBJ)**

```bash
ros2 topic pub -1 /target_object_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"


```

* **3. Grasping Completed, Proceed to Placement Box (APPROACHING_BOX)**

```bash
ros2 topic pub -1 /status/going_to_box std_msgs/msg/Bool "{data: true}"
ros2 topic pub -1 /target_box_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -1.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}}"


```

* **4. Emergency Stop / Abort Task (HALT)**

```bash
ros2 topic pub -1 /status/moving std_msgs/msg/Bool "{data: false}"


```

* **5. Monitor Arrival Feedback (Feedback)**

```bash
ros2 topic echo /move_feedback


```

## 🌟 Core Features & Optimization Summary

This project has undergone deep troubleshooting and custom optimization for four-wheel Skid-Steering robots and the Gazebo simulation environment:

1. **Completely solved map drift and ghosting**: By increasing the chassis equivalent wheelbase in URDF, upgrading the lidar scan frequency (20Hz / 720 samples), and squeezing the minimum update threshold of SLAM Toolbox (`0.05 rad`), the accumulated errors caused by rotating in place have been eliminated.
2. **RPP planner "smooth cornering"**: Disabled `use_rotate_to_heading` in the Nav2 RPP local planner, forcing the rover to use smooth arcs for cornering, thus avoiding TF tree oscillation caused by grinding in place.
3. **Eliminated lidar wall-hugging blind spot**: Lowered the `min_range` of the GPU Lidar from the default 0.35m to 0.10m to prevent the costmap from being accidentally cleared when closely hugging a wall, which could lead to chassis collisions.
4. **Unified Base Frame height**: Forced the reference coordinate system for SLAM and Nav2 to be `base_footprint` (Z=0), and set the spawn height to `Z=0.02`, perfectly solving the issue where the map was generated on the car roof in RViz.

```

```