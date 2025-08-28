#!/usr/bin/env python3
"""
Robot A Controller - UR5e with Gripper
Handles object pickup, transfer, and motion pattern execution
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import math
from scipy.spatial.transform import Rotation
import time
from enum import Enum

class TaskState(Enum):
    IDLE = 0
    PICKUP = 1
    TRANSFER = 2
    MOTION_PATTERN = 3
    COMPLETED = 4

class RobotAController(Node):
    def __init__(self):
        super().__init__('robot_a_controller')
        
        # Publishers (namespaced: tablea_ur)
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, '/tablea_ur/joint_trajectory_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/tablea_ur/gripper_command', 10)
        self.task_status_pub = self.create_publisher(String, '/task_status', 10)
        
        # Subscribers (namespaced: tablea_ur)
        self.joint_state_sub = self.create_subscription(
            JointState, '/tablea_ur/joint_states', self.joint_state_callback, 10)
        
        # Robot state
        self.current_joints = None
        self.joint_names = [
            'ur_shoulder_pan_joint',
            'ur_shoulder_lift_joint',
            'ur_elbow_joint',
            'ur_wrist_1_joint',
            'ur_wrist_2_joint',
            'ur_wrist_3_joint'
        ]
        
        # Task management
        self.current_state = TaskState.IDLE
        self.motion_start_time = None
        
        # Predefined poses (joint angles in radians)
        self.home_pose = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.pickup_pose = [-0.5, -1.2, 1.8, -2.1, -1.57, 0.0]
        self.transfer_pose = [0.0, -1.0, 1.3, -1.8, -1.57, 0.0]
        self.motion_center_pose = [0.2, -0.8, 1.0, -1.5, -1.57, 0.0]
        
        # Motion pattern parameters
        self.pattern_radius = 0.15
        self.pattern_frequency = 0.5
        self.perpendicular_amplitude = 0.05
        self.perpendicular_frequency = 0.3
        
        # Control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Robot A Controller initialized. Waiting for joint states...")
        
        status_msg = String()
        status_msg.data = "ROBOT_A_READY"
        self.task_status_pub.publish(status_msg)

    def joint_state_callback(self, msg):
        if len(msg.name) >= 6:
            joint_positions = [0.0] * 6
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    joint_positions[i] = msg.position[idx]
            self.current_joints = joint_positions
            
    def publish_joint_trajectory(self, target_joints, duration=3.0):
        if self.current_joints is None:
            self.get_logger().warn("No current joint state available")
            return
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        point1 = JointTrajectoryPoint()
        point1.positions = self.current_joints
        point1.time_from_start.sec = 0
        point1.time_from_start.nanosec = 0
        point2 = JointTrajectoryPoint()
        point2.positions = target_joints
        point2.time_from_start.sec = int(duration)
        point2.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points = [point1, point2]
        self.joint_trajectory_pub.publish(msg)
        
    def control_gripper(self, open_gripper=True):
        msg = Float64MultiArray()
        msg.data = [0.02, 0.02] if open_gripper else [-0.02, -0.02]
        self.gripper_pub.publish(msg)
        
    def forward_kinematics(self, joint_angles):
        x = 0.8 * (math.cos(joint_angles[0]) * math.cos(joint_angles[1] + joint_angles[2]))
        y = 0.8 * (math.sin(joint_angles[0]) * math.cos(joint_angles[1] + joint_angles[2]))
        z = 0.6 + 0.8 * math.sin(joint_angles[1] + joint_angles[2])
        return [x, y, z]
        
    def inverse_kinematics_approximate(self, target_pos, base_joints=None):
        if base_joints is None:
            base_joints = self.motion_center_pose
        target_joints = base_joints.copy()
        target_joints[0] = math.atan2(target_pos[1], target_pos[0])
        r = math.sqrt(target_pos[0]**2 + target_pos[1]**2)
        target_joints[1] = math.atan2(target_pos[2] - 0.6, r) - 0.2
        target_joints[2] = 1.0 + 0.3 * math.sin(target_joints[1])
        return target_joints
        
    def generate_motion_pattern(self, t):
        pattern_type = int(t / 10.0) % 2
        if pattern_type == 0:
            x_offset = self.pattern_radius * math.cos(2 * math.pi * self.pattern_frequency * t)
            z_offset = self.pattern_radius * math.sin(2 * math.pi * self.pattern_frequency * t)
        else:
            x_offset = self.pattern_radius * math.cos(2 * math.pi * self.pattern_frequency * t)
            z_offset = (self.pattern_radius / 2) * math.sin(4 * math.pi * self.pattern_frequency * t)
        y_offset = self.perpendicular_amplitude * math.sin(2 * math.pi * self.perpendicular_frequency * t)
        base_pos = self.forward_kinematics(self.motion_center_pose)
        return [base_pos[0] + x_offset, base_pos[1] + y_offset, base_pos[2] + z_offset]
        
    def control_loop(self):
        if self.current_joints is None:
            return
        current_time = time.time()
        if self.current_state == TaskState.IDLE:
            self.get_logger().info("Starting pickup sequence...")
            self.publish_joint_trajectory(self.home_pose, 2.0)
            self.control_gripper(True)
            self.current_state = TaskState.PICKUP
            self.state_start_time = current_time
        elif self.current_state == TaskState.PICKUP:
            if current_time - self.state_start_time > 3.0:
                self.get_logger().info("Moving to pickup position...")
                self.publish_joint_trajectory(self.pickup_pose, 3.0)
                self.state_start_time = current_time
            elif current_time - self.state_start_time > 6.0:
                self.get_logger().info("Closing gripper to pick object...")
                self.control_gripper(False)
                self.current_state = TaskState.TRANSFER
                self.state_start_time = current_time
        elif self.current_state == TaskState.TRANSFER:
            if current_time - self.state_start_time > 2.0:
                self.get_logger().info("Transferring object to Robot B's view...")
                self.publish_joint_trajectory(self.transfer_pose, 4.0)
                self.state_start_time = current_time
            elif current_time - self.state_start_time > 6.0:
                self.get_logger().info("Starting motion pattern execution...")
                self.publish_joint_trajectory(self.motion_center_pose, 3.0)
                self.current_state = TaskState.MOTION_PATTERN
                self.motion_start_time = current_time
                status_msg = String()
                status_msg.data = "MOTION_PATTERN_STARTED"
                self.task_status_pub.publish(status_msg)
        elif self.current_state == TaskState.MOTION_PATTERN:
            if self.motion_start_time is None:
                self.motion_start_time = current_time
            motion_time = current_time - self.motion_start_time
            if motion_time < 60.0:
                target_pos = self.generate_motion_pattern(motion_time)
                target_joints = self.inverse_kinematics_approximate(target_pos)
                self.publish_joint_trajectory(target_joints, 0.2)
            else:
                self.get_logger().info("Motion pattern completed!")
                self.current_state = TaskState.COMPLETED
                status_msg = String()
                status_msg.data = "MOTION_PATTERN_COMPLETED"
                self.task_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotAController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
