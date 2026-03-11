# Robot Control System

This project implements an end-to-end ROS 2 solution for autonomous “search–pick–place” with decoupled microservices. The **Main Controller** governs a strict finite‑state machine (FSM), while **Vision**, **Navigation**, and **Manipulator** act as focused actuators. Semantic memory is maintained per color, enabling the robot to pair objects with their matching boxes and complete **3 full cycles** before automatic shutdown.


## 1 System Architecture

All components are decoupled and communicate via ROS 2 topics and the Nav2 action server.

| Node (Entry Point)  | Runtime Name               | Responsibility                                                                                                                                                             |
| :------------------ | :------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **robot_fsm** | `robot_fsm`       | **Brain & FSM.** Maintains color-indexed memory (camera & map), transforms detections to `map` frame, dispatches Nav/Arm commands, and handles all state transitions.      |
| **camera_node**          | `camera_node`              | **Eyes.** RealSense frames → YOLOv8 detections (top‑k). Publishes color & type (`object` / `box`) with 3D coordinates in `camera_link`. Publishes a detection health flag. |
| **nav_node**      | `nav_node`      | **Legs.** Thin Nav2 ActionClient. Consumes `/goal_pose` and publishes `/nav/goal_reached` when Nav2 reports success. Handles preemption.                                   |
| **manipulator_node**     | `manipulator_node` | **Arm & Gripper.** Consumes `/arm/grasp_pose` and `/arm/grasp_status`. Sends target coords to MyCobot 280 and toggles gripper.                                             |

> **TF Requirement:** The controller transforms `camera_link` points to the `map` frame and **expects a valid TF tree**. No TF simulation node is provided here.


## 2 Finite State Machine (FSM)

The robot repeats the full mission **three times** and then shuts down.

### State Flow

1.  **INIT**
    *   **Logic:** Hardware graph probe (Vision, Nav, Arm).
    *   **Transition:** When all required pubs/subs are alive → **SEARCH**.

2.  **SEARCH**
    *   **Logic:** Publish random patrol goals in `map`. Build **color → {object, box}** memory by transforming camera detections to `map`.
    *   **Transition:** When a color has **both** `object` and `box` in memory → set `active_target_color` → **MOVE\_TO\_OBJECT**.

3.  **MOVE\_TO\_OBJECT**
    *   **Logic:** Navigate to the memorized `map` pose of the target **object**.
    *   **Transition:** On `/nav/goal_reached == True` → **GRASP**.

4.  **GRASP**
    *   **Logic:** Halt chassis. Send **`camera_link`-relative** gripper pose to pick the **object**.
    *   **Transition:** When `/detection_state == False` (object no longer detected) → **MOVE\_TO\_BOX**.

5.  **MOVE\_TO\_BOX**
    *   **Logic:** Navigate to the memorized `map` pose of the matching **box**.
    *   **Transition:** On `/nav/goal_reached == True` → **DROP**.

6.  **DROP**
    *   **Logic:** Halt chassis. Send **`camera_link`-relative** gripper pose to place the **object** into the **box**.
    *   **Transition:**
        *   If **cycles == 3** → shutdown.
        *   Else: clear memory for the completed color, reset `active_target_color = "NONE"` → **SEARCH**.

### Output Control Matrix (as-implemented)

| STATE            | CHASSIS\_CMD | ARM\_TASK   | VISION\_MODE | TF\_FRAME     | TARGET\_TYPE |
| :--------------- | :----------- | :---------- | :----------- | :------------ | :----------- |
| INIT             | `IDLE`       | `RESET`     | `STANDBY`    | `NONE`        | `NONE`       |
| SEARCH           | `EXPLORE`    | `STOW`      | `SCAN_ALL`   | `map`         | `any`        |
| MOVE\_TO\_OBJECT | `NAVIGATE`   | `STOW`      | `LOCK_COLOR` | `map`         | `object`     |
| GRASP            | `BRAKE`      | `PICK_ACT`  | `LOCK_COLOR` | `camera_link` | `object`     |
| MOVE\_TO\_BOX    | `NAVIGATE`   | `HOLD_OBJ`  | `LOCK_COLOR` | `map`         | `box`        |
| DROP             | `BRAKE`      | `PLACE_ACT` | `LOCK_COLOR` | `camera_link` | `box`        |

***

## 3 Communication API (Topics & Action)

### Controller → Actuators (Outbound)

*   **`/goal_pose`** — `geometry_msgs/PoseStamped`  
    Target pose in **`map`**. Used by Navigation.
*   **`/arm/grasp_pose`** — `my_robot_interfaces/GripperPose`  
    Target **`camera_link`**-relative grasp/place pose for Manipulator.
*   **`/arm/grasp_status`** — `my_robot_interfaces/GripperState`  
    Gripper command (`grip: True` to close, `False` to open).
*   **`/vision/active_color`** — `std_msgs/String`  
    Broadcasts the currently tracked color (optional; Vision node does **not** consume this in current code).

### Actuators/Vision → Controller (Inbound)

*   **`/detected_object`** — `my_robot_interfaces/ObjectTarget`  
    YOLO detection in **camera coordinates** with `{name, color, x, y, z}`:
    *   `name ∈ {"object", "box"}`
    *   `color ∈ {"red", "yellow", "purple", "unknown"}`
*   **`/detection_state`** — `std_msgs/Bool`  
    **True** when any YOLO detections exist in a frame; **False** otherwise.
    > Used as **negative confirmation**: `False` during **GRASP** means object disappeared (picked); `False` during **DROP** is used to finalize a cycle.
*   **`/nav/goal_reached`** — `std_msgs/Bool`  
    **True** when Nav2 reports **succeeded**.

### Action Server (Navigation)

*   **`navigate_to_pose`** — `nav2_msgs/action/NavigateToPose`  
    Used internally by `nav_controller_node` (preemption supported).

***

## 4 Prerequisites

*   **ROS 2 Jazzy** (Ubuntu 24.04 recommended)
*   **Nav2** stack (start your usual bringup for the robot/map)
*   **Python libraries**:
    *   `ultralytics` (YOLOv8)
    *   `numpy`, `opencv-python`
    *   `pyrealsense2` (Intel RealSense)
    *   `pymycobot` (MyCobot 280)
*   **Custom Interfaces** (`my_robot_interfaces` must be built first):
    *   `ObjectTarget.msg`
    *   `GripperState.msg`
    *   `GripperPose.msg`
*   **Model file:** Place **`best.pt`** inside the **installed share** of `robot_control_system`. Ensure `setup.py` installs this file under `data_files` so it ends up in the package share.


## 5 Build Instructions

```bash
# Build custom interfaces first
cd ~/ros2_ws
colcon build --packages-select my_robot_interfaces
source install/setup.bash

# Then build the main system
colcon build --packages-select robot_control_system
source install/setup.bash
```


## 6 Launching & Testing

### Launch

```bash
ros2 launch robot_control_system system_control.launch.py
```

**Required services running before/alongside:**

*   **TF tree** providing `map → ... → camera_link`
*   **Nav2** (e.g., `bringup`) so the action server `navigate_to_pose` is active
*   **Manipulator hardware** (MyCobot 280) connected and accessible

### Manual Simulation / Verification (No Hardware)

You can verify FSM transitions using CLI publishers. **Start all four nodes** so the controller passes `INIT` hardware checks (or spoof the required pubs/subs).

**1) Seed detections (Vision → Controller)**  
**Publish an object** (camera coordinates, meters):

```bash
ros2 topic pub /detected_object my_robot_interfaces/msg/ObjectTarget \
"{name: object, color: red, x: 0.35, y: 0.02, z: 0.48}" --once
```

**Publish the matching box**:

```bash
ros2 topic pub /detected_object my_robot_interfaces/msg/ObjectTarget \
"{name: box, color: red, x: 0.90, y: -0.10, z: 0.60}" --once
```

**Keep detection state True while vision sees items:**

```bash
ros2 topic pub /detection_state std_msgs/msg/Bool "{data: true}"
```

**2) Simulate Nav arrival (Nav → Controller)**  
Triggers **GRASP** or **DROP** depending on current state:

```bash
ros2 topic pub /nav/goal_reached std_msgs/msg/Bool "{data: true}" --once
```

**3) Simulate grasp/drop confirmation (Vision → Controller)**  
The controller uses **False** as success (no detections):

```bash
# After sending a grasp pose, confirm "object disappeared"
ros2 topic pub /detection_state std_msgs/msg/Bool "{data: false}" --once

# After sending a place pose, confirm "clear scene" to finalize the cycle
ros2 topic pub /detection_state std_msgs/msg/Bool "{data: false}" --once
```

Repeat the above sequence to complete up to **3 cycles**. On the third drop confirmation, the controller shuts down.


## 7) Project Structure

```text
.
├── my_robot_interfaces
│   ├── msg/
│   │   ├── ObjectTarget.msg
│   │   ├── GripperState.msg
│   │   ├── ErrorStatus.msg
│   │   └── GripperPose.msg
│   ├── package.xml
│   └── CMakeLists.txt
└── robot_control_system
    ├── robot_control_system/
    │   ├── robot_fsm.py          # Main FSM (Brain)
    │   ├── nav_node.py           # Nav2 ActionClient (Legs)
    │   ├── camera_node.py        # RealSense + YOLOv8 (Eyes)
    │   ├── manipulator_node.py   # MyCobot 280 control (Arm)
    │   ├── best.pt               # YOLOv8 weights
    │   └── __init__.py
    ├── launch/
    │   └── system_control.launch.py
    ├── README.md
    ├── setup.py
    └── package.xml
```


## 8 Important Notes & Troubleshooting

### TF & Frames

*   The controller **requires** valid TF from `camera_link` to `map`. If running without a full robot stack, provide a static transform (example):
    ```bash
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_link
    ```
*   Vision publishes detections in **`camera_link`** (meters). The controller transforms to `map` for navigation targets.

### Nav2 Bringup

*   `nav_controller_node` **waits** for `navigate_to_pose`. If not running, goals won’t be accepted and `/nav/goal_reached` won’t publish. Start your standard Nav2 bringup and ensure localization is active.

### RealSense & YOLO

*   Ensure `pyrealsense2` is installed and the device is connected. The Vision node runs at \~10 Hz and sets `/detection_state` to **False** if **no boxes** are present in a frame.
*   Place `best.pt` in the installed package **share** (see Prerequisites). If not found, the Vision node will not start.

### Manipulator (MyCobot 280)

*   The manipulator currently **sends coordinates exactly as received** from the FSM:
    *   FSM publishes **meters** (from RealSense).
    *   **MyCobot’s `send_coords` expects millimeters.**  
        **⚠️ You must reconcile units** (e.g., scale `x,y,z` by `1000.0`) inside `manipulator_control_node.py` or adjust upstream to match hardware requirements.
*   Gripper mapping:
    *   `grip == True` → `arm.set_gripper_state(0, 100)` (close)
    *   `grip == False` → `arm.set_gripper_state(1, 100)` (open)
        Confirm this mapping matches your end-effector.

### Detection Logic Semantics

*   The controller uses **`/detection_state == False`** as the **success signal** for both **GRASP** and **DROP**. If your scene never goes fully blank (e.g., other distractor objects), you may need a more specific signal (e.g., track-lost for the **active color** only).

### Hardware Readiness (INIT)

*   INIT passes when:
    *   Vision publishes `/detection_state`
    *   Nav subscribes to `/goal_pose`
    *   Manipulator subscribes to `/arm/grasp_pose`
*   If stuck in INIT, verify these pubs/subs exist (`ros2 topic list`, `ros2 topic info ...`).



## 9 Quick Reference – Topic Cheatsheet

```text
# Controller publishes:
/goal_pose                  geometry_msgs/PoseStamped
/arm/grasp_pose             my_robot_interfaces/GripperPose
/arm/grasp_status           my_robot_interfaces/GripperState
/vision/active_color        std_msgs/String

# Controller subscribes:
#/detected_object           my_robot_interfaces/ObjectTarget
#/detection_state           std_msgs/Bool
#/nav/goal_reached          std_msgs/Bool

# Nav2 action server:
navigate_to_pose            nav2_msgs/action/NavigateToPose
```


## 10 Safety & Best Practices

*   **Test in simulation** (or with the arm powered but lifted) before enabling gripper & motion.
*   **Clamp reach & z-depth** in the manipulator node to protect the end-effector.
*   Validate TF alignment between **camera** and **arm** before attempting real picks/places.
