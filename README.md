# AERO62520 – Robotic Systems Design Project (Team 5)
### Autonomous Mobile Manipulation System

This repository contains the software developed by **Team 5** for the AERO62520 Robotic Systems Design Project.  
The system enables a mobile robot to **autonomously explore an unknown indoor environment, detect coloured target objects, grasp them using a 6-DOF manipulator, and deposit them into the correct bin**.

The software architecture is implemented in **ROS2 Jazzy** and distributed across three computing units:

- **Intel NUC** – high-level autonomy (SLAM, perception, navigation, IK, mission logic)  
- **Leo Rover Raspberry Pi** – base motor control and odometry  
- **myCobot 280 Pi Raspberry Pi** – arm actuation and gripper control  

---

## Repository Contents

The repository is organised into the primary subsystems developed by the team:

### 1. SLAM & Localisation (NUC)
ROS2 SLAM Toolbox integration using the A2M12 LiDAR and wheel odometry to build a 2D map and estimate robot pose.

### 2. Vision Perception
YOLOv8s–based object and bin detection, trained on a custom dataset containing red, yellow, and purple objects and bins.  
Depth processing computes 3D object positions for navigation and grasp planning.

### 3. Navigation (Nav2)
Global and local planning for:
- autonomous exploration,  
- approaching detected objects,  
- navigating to known bin locations,  
- returning to the start position.

### 4. Manipulation
Inverse kinematics computed on the NUC;  
Joint and gripper execution handled on the myCobot Pi using the `pymycobot` API.

### 5. Base Control (Leo Rover Pi)
Real-time motor control, odometry publishing, and execution of `/cmd_vel` commands.  
Also manages emergency stop and safety behaviours.

### 6. Mission Logic
High-level state machine coordinating exploration, detection, navigation, grasping, deposition, and mission completion.

---



More updates will be pushed as integration and testing continue.

---

## Project Goal
To produce a robot capable of completing the full **retrieve → transport → deposit** mission autonomously using perception, SLAM, navigation, and manipulation modules developed by the team.

---

## How to Use the Repository
Full system run instructions will be added once integration and testing are complete.  

---


