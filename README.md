## Visual-guided-Dual-Arm-Synchronization-with-Decentralized-Control

## Project Overview
This project implements a **dual-arm robotic system simulation** using **ROS 2 Humble, MoveIt2, and Gazebo**. 

Two collaborative robots are deployed:  
- **Robot A (UR 6-DoF + Gripper)** – responsible for **object pickup, transfer, and executing cyclic trajectories** (circular, Lissajous, and sinusoidal).  
- **Robot B (Franka Panda 7-DoF + Wrist-Mounted Camera)** – performs **visual tracking** of Robot A’s end-effector and maintains a desired relative pose through **joint-space control**.  

The setup emphasizes **decentralized control**, where Robot B relies solely on **camera feedback** and does not access Robot A’s joint states or planned trajectories.

## Objectives
- Develop a **dual-arm simulation framework** in ROS2 Humble and Gazebo.  
- Implement **object-picking and trajectory execution** for Robot A (UR).  
- Achieve **visual-guided tracking** using Robot B (Franka Panda with camera).  
- Ensure **decentralized synchronization** (no state sharing between robots).  

## Approach
1. **Simulation Setup**: Created a Gazebo world with two tables and mounted robots (UR + Panda).  
2. **Motion Planning (UR)**: Used MoveIt2 to generate collision-free pickup and trajectory paths.  
3. **Trajectory Execution**: Designed circular, Lissajous, and sinusoidal patterns for UR motion.  
4. **Visual Tracking (Panda)**: Wrist-mounted camera captured UR’s end-effector; pose estimated via vision.  
5. **Decentralized Control**: Panda controlled in joint space based only on visual feedback.  

## Techniques Used
- **Motion Planning (UR Arm)**  
  - MoveIt2 for collision-free trajectory generation.  
  - Cyclic trajectory execution (Circle, Lissajous, Sinusoidal).  

- **Visual Tracking (Panda)**  
  - Pose estimation using camera feed + feature/marker detection.  
  - Camera calibration for accurate 3D transforms.  

- **Control Strategies**  
  - Joint-space control for tracking target pose.  
  - Redundancy handling via null-space optimization.  
  - Decentralized control (vision only, no shared states).  

- **Simulation Tools**  
  - Gazebo for environment + physics simulation.  
  - RViz for visualization.  
  - ROS2 Humble middleware.  

## Workspace Structure
```
ws_dualarm/
│── src/
│   ├── dualarm_description/     # URDF/Xacro files for UR + Panda
│   ├── dualarm_bringup/         # Launch files for Gazebo + RViz
│   ├── dualarm_control/         # ROS2 Control + Controllers configs
│   ├── dualarm_moveit_config/   # MoveIt2 planning setup
│── README.md                    # Documentation
```

## 💻 Commands to Run (Replication)
### 1) Clone workspace and build:
```bash
# 1) Build & source
cd ~/ws_dualarm
colcon build --merge-install
source install/setup.bash
```
### 2) Ensure GZ resources (one time per shell)
```bash
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:\
$(ros2 pkg prefix dualarm_description)/share:\
$(ros2 pkg prefix ur_description)/share:\
$(ros2 pkg prefix franka_description)/share"
```
### 3) Launch (spawns URDF with -urdf inside)
```bash
ros2 launch dualarm_control integrated_system.launch.py
```
### 4) Start controllers if not auto-spawned (example)
```bash
ros2 run controller_manager spawner -c /tablea_ur/controller_manager joint_state_broadcaster
ros2 run controller_manager spawner -c /tablea_ur/controller_manager scaled_joint_trajectory_controller
ros2 run controller_manager spawner -c /tableb_panda/controller_manager joint_state_broadcaster
ros2 run controller_manager spawner -c /tableb_panda/controller_manager joint_trajectory_controller
```
### 5) Run coordinator (if separated) – starts pick→place→patterns
```bash
ros2 run dualarm_control system_coordinator
```
### 6) Check controllers:
```bash
ros2 control list_controllers -c /tablea_ur/controller_manager
ros2 control list_controllers -c /tableb_panda/controller_manager
```

## Results & Observations
- UR arm successfully executed **pick-and-place** + cyclic patterns.  
- Panda tracked UR’s end-effector with **low tracking error (~5–10 mm)**.  
- Decentralized architecture ensured robustness to communication loss.  
- Challenges: sensitivity to visual noise, lighting, and FOV.  
