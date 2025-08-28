#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

UR_NS = "tablea_ur"        # Robot A (UR)
PANDA_NS = "tableb_panda"  # Robot B (Panda)

UR_URDF    = os.path.expanduser("~/.ros/tablea_ur.urdf")
PANDA_URDF = os.path.expanduser("~/.ros/tableb_panda.urdf")

def read_file(p):
    with open(p, "r") as f:
        return f.read()

def generate_launch_description():
    bringup = os.popen("ros2 pkg prefix dualarm_bringup").read().strip()
    if not bringup:
        raise RuntimeError("Package dualarm_bringup not found. Did you `source install/setup.bash`?")

    world_sdf = os.path.join(bringup, "share/dualarm_bringup/worlds/dual_table.sdf")

    # Sanity checks
    for p in [UR_URDF, PANDA_URDF, world_sdf]:
        if not os.path.exists(p) or os.path.getsize(p) == 0:
            raise RuntimeError(f"Required file missing/empty: {p}")

    # 1) Start Gazebo world (tables only)
    gz_sim = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", world_sdf, "--force-version", "6"],
        output="screen",
    )

    # 2) Robot state publishers fed from the same URDF files
    rsp_ur = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=UR_NS,
        output="screen",
        parameters=[{"robot_description": read_file(UR_URDF)}],
    )
    rsp_panda = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=PANDA_NS,
        output="screen",
        parameters=[{"robot_description": read_file(PANDA_URDF)}],
    )

    # 3) Spawn robots FROM FILES (no topic dependency)
    spawn_ur = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_sim", "create",
            "-name", "tablea_ur", "-urdf", "-urdf",
            "-file", UR_URDF,
            "--ros-args", "-r", f"__ns:=/{UR_NS}",
        ],
        output="screen",
    )
    spawn_panda = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_sim", "create",
            "-name", "tableb_panda", "-urdf",
            "-file", PANDA_URDF, "-urdf", "-file", PANDA_URDF,
            "--ros-args", "-r", f"__ns:=/{PANDA_NS}",
        ],
        output="screen",
    )

    # 4) Controllers
    ur_jsb = Node(package="controller_manager", executable="spawner", namespace=UR_NS,
                  arguments=["joint_state_broadcaster", "--controller-manager", f"/{UR_NS}/controller_manager"],
                  output="screen")
    ur_traj = Node(package="controller_manager", executable="spawner", namespace=UR_NS,
                   arguments=["joint_trajectory_controller", "--controller-manager", f"/{UR_NS}/controller_manager"],
                   output="screen")
    panda_jsb = Node(package="controller_manager", executable="spawner", namespace=PANDA_NS,
                     arguments=["joint_state_broadcaster", "--controller-manager", f"/{PANDA_NS}/controller_manager"],
                     output="screen")
    panda_traj = Node(package="controller_manager", executable="spawner", namespace=PANDA_NS,
                      arguments=["joint_trajectory_controller", "--controller-manager", f"/{PANDA_NS}/controller_manager"],
                      output="screen")

    # 5) Camera bridge + MoveIt (SRDF is dualarm.xml, not .srdf)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            f"/{PANDA_NS}/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            f"/{PANDA_NS}/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
    )
    moveit = ExecuteProcess(
        cmd=["ros2", "launch", "dualarm_moveit_config", "moveit.launch.py", "srdf_filename:=dualarm.xml"],
        output="screen",
    )

    # simple timing
    t1 = TimerAction(period=1.0, actions=[gz_sim])
    t2 = TimerAction(period=3.0, actions=[rsp_ur, rsp_panda])
    t3 = TimerAction(period=5.0, actions=[spawn_ur, spawn_panda])
    t4 = TimerAction(period=9.0, actions=[ur_jsb, ur_traj, panda_jsb, panda_traj])
    t5 = TimerAction(period=12.0, actions=[bridge, moveit])

    return LaunchDescription([t1, t2, t3, t4, t5])
