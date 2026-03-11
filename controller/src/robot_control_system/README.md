# Robot Control System

This project implements an end-to-end ROS 2 solution for autonomous “search–pick–place” with decoupled microservices. The **Main Controller** governs a strict finite‑state machine (FSM), while **Vision**, **Navigation**, and **Manipulator** act as focused actuators. Semantic memory is maintained per color, enabling the robot to pair objects with their matching boxes and complete **3 full cycles** before automatic shutdown.

## 1 System Architecture

All components are decoupled and communicate via ROS 2 topics and the Nav2 action server. The FSM manages logic (Control Flow), while actuators handle the physical execution and coordinate synthesis (Data Flow).

| Node                 | Runtime Name               | Responsibility                                                                                                                                                                                           |
| :------------------- | :------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **robot_fsm** | `robot_control_node`       | **Brain & FSM.** Maintains global `map` coordinates. Publishes boolean triggers. Dynamically resolves `camera_link` coordinates for the arm at the exact moment of action to eliminate movement errors. |
| **camera_node** | `vision_node`              | **Eyes.** RealSense frames → YOLOv8 detections (top‑k). Publishes color & type (`object`/`box`) with 3D coordinates in `camera_link`. Publishes a detection health flag (`/detection_state`).            |
| **nav_node** | `nav_controller_node`      | **Smart Legs.** Consumes `/nav/goal_point` (`PointStamped`). Automatically calculates the required Yaw to face the target and stops at a safe **0.25m standoff distance** to allow arm operation.        |
| **manipulator_node** | `manipulator_control_node` | **Arm & Gripper.** Consumes `/arm/grasp_pose` and `/arm/grasp_status`. Executes open-loop physical movements on the MyCobot 280.                                                                         |

> **TF Requirement:** The controller dynamically transforms `camera_link` points to the `map` frame (for memory) and back to `camera_link` (for manipulation). A valid TF tree (`map` -> `odom` -> `base_link` -> `camera_link`) is strictly required.



## 2 Finite State Machine (FSM)

The robot repeats the full mission **three times** and then shuts down. Feedback from the open-loop arm is simulated via internal timers and verified robustly via vision timestamps.

### State Flow

1.  **INIT**
    * **Logic:** Hardware graph probe. Checks if Vision, Nav, and Arm publishers/subscribers are alive.
    * **Transition:** When all required nodes are online → **SEARCH**.

2.  **SEARCH**
    * **Logic:** Trigger Nav node to explore randomly (bounds: [-1.5, 1.5]). Build **color → {object, box}** memory by transforming real-time camera detections to `map`.
    * **Transition:** When a color has **both** `object` and `box` in memory → set `active_target_color` → **MOVE_TO_OBJECT**.

3.  **MOVE_TO_OBJECT**
    * **Logic:** Pass the target object's `map` point to Nav. Nav auto-calculates orientation and standoff distance.
    * **Transition:** On `/nav/goal_reached == True` → **GRASP**.

4.  **GRASP**
    * **Logic:** FSM looks up the latest TF to transform the object's `map` pose back to a highly accurate `camera_link` pose. Send to Arm. FSM waits **5.0 seconds** for physical execution.
    * **Transition:** After 5s, check vision memory timestamp.
        * If the object was seen in the exact location < 1.5s ago → Grasp failed, retry.
        * If not seen recently → Grasp success → **MOVE_TO_BOX**.

5.  **MOVE_TO_BOX**
    * **Logic:** Pass the target box's `map` point to Nav.
    * **Transition:** On `/nav/goal_reached == True` → **DROP**.

6.  **DROP**
    * **Logic:** Send `camera_link`-relative box pose to Arm. FSM waits **3.0 seconds** for physical execution.
    * **Transition:**
        * If **cycles == 3** → shutdown.
        * Else: Clear memory for the completed color, reset `active_target_color = "NONE"` → **SEARCH**.



### Output Control Matrix

The FSM drives the entire system using this highly optimized 4-variable boolean matrix:

| STATE                | NAV_EXPLORE | NAV_GOTO | ARM_GRASP | ARM_DROP |
| :------------------- | :---------- | :------- | :-------- | :------- |
| **[0] INIT** | False       | False    | False     | False    |
| **[1] SEARCH** | True        | False    | False     | False    |
| **[2] MOVE_OBJECT** | False       | True     | False     | False    |
| **[3] GRASP** | False       | False    | True      | False    |
| **[4] MOVE_BOX** | False       | True     | False     | False    |
| **[5] DROP** | False       | False    | False     | True     |


## 3 Communication API (Topics & Interfaces)

### Controller → Actuators (Outbound)

* **`/nav/goal_point`** — `geometry_msgs/PointStamped`
    Target `x, y` point in **`map`** frame. Nav node uses this to synthesize yaw and the 0.25m offset.
* **`/nav/cmd_explore`** — `std_msgs/Bool`
    Triggers infinite random waypoint exploration in Nav node (`True`=Explore, `False`=Brake/Stop).
* **`/arm/grasp_pose`** — `my_robot_interfaces/GripperPose`
    Target **`camera_link`**-relative pose for the Manipulator (`x, y, z, roll, pitch, yaw`).
* **`/arm/grasp_status`** — `my_robot_interfaces/GripperState`
    Gripper command (`grip: True` to close/pick, `False` to open/place).

### Actuators/Vision → Controller (Inbound)

* **`/detected_object`** — `my_robot_interfaces/ObjectTarget`
    YOLO detection published by Vision in **camera coordinates** with `{name, color, x, y, z}`.
* **`/nav/goal_reached`** — `std_msgs/Bool`
    **True** when Nav2 reports successful arrival at the `GOTO` target.

*(Note: The Vision node also publishes `/detection_state` (`std_msgs/Bool`), but the FSM uses the precise timestamps of `/detected_object` to perform robust closed-loop grasp verification instead).*

### Action Server (Internal to Navigation)

* **`Maps_to_pose`** — `nav2_msgs/action/NavigateToPose`
    Used by the `nav_controller_node` to execute both random exploration points and synthesized FSM targets.


## 4 Prerequisites

* **ROS 2 Jazzy** (Ubuntu 24.04 recommended)
* **Nav2** stack (start your usual bringup so `Maps_to_pose` is active)
* **Python libraries**:
    * `ultralytics` (YOLOv8)
    * `numpy`, `opencv-python`
    * `pyrealsense2` (Intel RealSense SDK)
    * `pymycobot` (MyCobot 280)
* **Custom Interfaces** (`my_robot_interfaces` must be built first):
    * `ObjectTarget.msg`
    * `GripperState.msg`
    * `GripperPose.msg`
* **Model file:** Place **`best.pt`** inside the installed share directory of `robot_control_system`.

## 5 Build Instructions

```bash
# Build custom interfaces first
cd ~/ros2_ws
colcon build --packages-select my_robot_interfaces
source install/setup.bash

# Then build the main system
colcon build --packages-select robot_control_system
source install/setup.bash