#!/usr/bin/env python3
"""
Robot B Controller - Panda with Camera
Handles visual tracking and synchronization with decentralized control
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import cv2
from cv_bridge import CvBridge
import math
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
import time
from enum import Enum

class TrackingState(Enum):
    WAITING = 0
    TRACKING = 1
    LOST = 2

class RobotBController(Node):
    def __init__(self):
        super().__init__('robot_b_controller')
        
        # Publishers (namespaced: tableb_panda)
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, '/tableb_panda/joint_trajectory_controller/joint_trajectory', 10)
        self.task_status_pub = self.create_publisher(String, '/task_status', 10)  # <-- missing in your file
        
        # Subscribers (namespaced: tableb_panda)
        self.joint_state_sub = self.create_subscription(
            JointState, '/tableb_panda/joint_states', self.joint_state_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/tableb_panda/camera/image_raw', self.camera_callback, 10)
        self.task_status_sub = self.create_subscription(
            String, '/task_status', self.task_status_callback, 10)
        
        # Robot state
        self.current_joints = None
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        
        # Vision processing
        self.bridge = CvBridge()
        self.latest_image = None
        self.object_detected = False
        self.object_position_2d = None
        self.object_position_3d = None
        
        # Tracking state
        self.tracking_state = TrackingState.WAITING
        self.last_detection_time = 0
        
        # Camera intrinsics (placeholder)
        self.camera_matrix = np.array([[640,0,320],[0,640,240],[0,0,1]], dtype=np.float32)
        self.object_size = 0.05  # meters
        
        # Control parameters
        self.desired_relative_pose = [0.3, 0.0, 0.2]
        self.tracking_gains = {'p': 2.0, 'd': 0.5}
        self.prev_error = None
        
        # Panda joint limits
        self.joint_limits = {
            'lower': [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
            'upper': [ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973]
        }
        
        # Poses
        self.home_pose = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self.ready_pose = [0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.785]
        
        # Control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Robot B Controller initialized. Waiting for camera data...")
        
        status_msg = String()
        status_msg.data = "ROBOT_B_READY"
        self.task_status_pub.publish(status_msg)

    # --- rest identical except topic names already changed above ---
    def joint_state_callback(self, msg):
        if len(msg.name) >= 7:
            joint_positions = [0.0] * 7
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    joint_positions[i] = msg.position[idx]
            self.current_joints = joint_positions

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            self.detect_object(cv_image)
        except Exception as e:
            self.get_logger().error(f"Camera processing error: {e}")

    def task_status_callback(self, msg):
        if msg.data == "MOTION_PATTERN_STARTED":
            self.get_logger().info("Motion pattern started - beginning tracking")
            self.tracking_state = TrackingState.TRACKING

    def detect_object(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 50, 50]); upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50]); upper_red2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 100:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"]); cy = int(M["m01"] / M["m00"])
                    self.object_position_2d = [cx, cy]; self.object_detected = True
                    self.last_detection_time = time.time()
                    self.estimate_3d_position([cx, cy])
                    cv2.circle(image, (cx, cy), 10, (0, 255, 0), -1)
                    cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
        else:
            self.object_detected = False
        if time.time() - self.last_detection_time > 0.5:
            self.tracking_state = TrackingState.LOST

    def estimate_3d_position(self, pixel_coords):
        if self.latest_image is None:
            return
        hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 50, 50]); upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50]); upper_red2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            f = self.camera_matrix[0, 0]
            size_px = max(w, h)
            estimated_depth = (self.object_size * f) / size_px
            u, v = pixel_coords
            x_cam = (u - self.camera_matrix[0, 2]) * estimated_depth / self.camera_matrix[0, 0]
            y_cam = (v - self.camera_matrix[1, 2]) * estimated_depth / self.camera_matrix[1, 1]
            z_cam = estimated_depth
            self.object_position_3d = [x_cam, y_cam, z_cam]

    def panda_forward_kinematics(self, q):
        x = 0.5 * math.cos(q[0]) * math.cos(q[1] + q[2])
        y = 0.5 * math.sin(q[0]) * math.cos(q[1] + q[2])
        z = 0.8 + 0.5 * math.sin(q[1] + q[2])
        return np.array([x, y, z])
        
    def jacobian_approximate(self, q):
        eps = 1e-6; J = np.zeros((3, 7)); base = self.panda_forward_kinematics(q)
        for i in range(7):
            q2 = q.copy(); q2[i] += eps
            J[:, i] = (self.panda_forward_kinematics(q2) - base) / eps
        return J
        
    def inverse_kinematics_with_redundancy(self, target_position, current_joints):
        from scipy.optimize import minimize
        def objective(q):
            pos_err = np.linalg.norm(self.panda_forward_kinematics(q) - target_position)
            penalty = 0
            for qi, lo, hi in zip(q, self.joint_limits['lower'], self.joint_limits['upper']):
                if qi < lo: penalty += 10*(lo-qi)**2
                elif qi > hi: penalty += 10*(qi-hi)**2
            reg = 0.1*np.linalg.norm(q-current_joints)
            return pos_err + penalty + reg
        bounds = [(lo, hi) for lo, hi in zip(self.joint_limits['lower'], self.joint_limits['upper'])]
        res = minimize(objective, current_joints, method='L-BFGS-B', bounds=bounds)
        return res.x if res.success else current_joints
            
    def compute_desired_end_effector_pose(self):
        if self.object_position_3d is None:
            return None
        return np.array(self.object_position_3d) + np.array(self.desired_relative_pose)
        
    def pd_control(self, current_pos, desired_pos, dt):
        error = desired_pos - current_pos
        p = self.tracking_gains['p'] * error
        d = np.zeros_like(error) if self.prev_error is None else self.tracking_gains['d'] * (error - self.prev_error) / dt
        self.prev_error = error
        return p + d
        
    def publish_joint_trajectory(self, target_joints, duration=0.1):
        if self.current_joints is None:
            return
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = target_joints.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(duration * 1e9)
        msg.points = [point]
        self.joint_trajectory_pub.publish(msg)
        
    def control_loop(self):
        if self.current_joints is None:
            return
        q = np.array(self.current_joints)
        if self.tracking_state == TrackingState.WAITING:
            self.publish_joint_trajectory(np.array(self.ready_pose), 1.0)
        elif self.tracking_state == TrackingState.TRACKING:
            if self.object_detected and self.object_position_3d is not None:
                desired = self.compute_desired_end_effector_pose()
                if desired is not None:
                    current = self.panda_forward_kinematics(q)
                    u = self.pd_control(current, desired, 0.05)
                    J = self.jacobian_approximate(q)
                    qdot = np.linalg.pinv(J) @ u
                    target = q + qdot * 0.05
                    target = np.clip(target, self.joint_limits['lower'], self.joint_limits['upper'])
                    self.publish_joint_trajectory(target, 0.1)
            else:
                self.get_logger().warn("Object not detected during tracking")
        elif self.tracking_state == TrackingState.LOST:
            self.get_logger().warn("Object tracking lost")
            self.publish_joint_trajectory(np.array(self.ready_pose), 2.0)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotBController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
