#!/usr/bin/env python3
"""
Enhanced Vision Processing Node
Provides advanced computer vision capabilities for object detection and tracking
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from scipy.spatial.transform import Rotation
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import String

# Custom message (you'd normally define this in a .msg file)
class ObjectDetection:
    def __init__(self):
        self.header = Header()
        self.object_id = ""
        self.confidence = 0.0
        self.bounding_box = []  # [x, y, width, height]
        self.center_2d = Point()
        self.position_3d = Point()
        self.pose_3d = Pose()

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')
        
        # Publishers
        self.debug_image_pub = self.create_publisher(Image, '/vision/debug_image', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/tableb_panda/camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/tableb_panda/camera/camera_info', self.camera_info_callback, 10)

        # Vision processing
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_detections = []
        
        # Object tracking
        self.trackers = {}
        self.tracking_id_counter = 0
        
        # Detection parameters
        self.detection_params = {
            'red_lower_hsv1': np.array([0, 50, 50]),
            'red_upper_hsv1': np.array([10, 255, 255]),
            'red_lower_hsv2': np.array([170, 50, 50]),
            'red_upper_hsv2': np.array([180, 255, 255]),
            'min_area': 200,
            'max_area': 10000,
            'min_contour_points': 8
        }
        
        # ArUco detector (for more robust tracking if available)
        try:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.use_aruco = True
        except:
            self.use_aruco = False
            self.get_logger().warn("ArUco not available, using color detection only")
        
        # Kalman filters for tracking
        self.kalman_filters = {}
        
        self.get_logger().info("Vision processor initialized")
        
        self.status_pub = self.create_publisher(String, '/task_status', 10)
        status_msg = String()
        status_msg.data = "VISION_READY"
        self.status_pub.publish(status_msg)


    def camera_info_callback(self, msg):
        """Update camera intrinsics"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        
    def create_kalman_filter(self):
        """Create Kalman filter for object tracking"""
        kf = cv2.KalmanFilter(4, 2)
        kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                        [0, 1, 0, 0]], np.float32)
        kf.transitionMatrix = np.array([[1, 0, 1, 0],
                                       [0, 1, 0, 1],
                                       [0, 0, 1, 0],
                                       [0, 0, 0, 1]], np.float32)
        kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.1
        return kf
        
    def detect_colored_objects(self, image):
        """Detect objects based on color"""
        detections = []
        
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create masks for red color
        mask1 = cv2.inRange(hsv, self.detection_params['red_lower_hsv1'], 
                           self.detection_params['red_upper_hsv1'])
        mask2 = cv2.inRange(hsv, self.detection_params['red_lower_hsv2'], 
                           self.detection_params['red_upper_hsv2'])
        mask = mask1 + mask2
        
        # Morphological operations to clean up mask
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if (self.detection_params['min_area'] < area < self.detection_params['max_area'] and
                len(contour) > self.detection_params['min_contour_points']):
                
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate center
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    detection = ObjectDetection()
                    detection.object_id = "red_object"
                    detection.confidence = min(area / 1000.0, 1.0)  # Normalize confidence
                    detection.bounding_box = [x, y, w, h]
                    detection.center_2d.x = float(cx)
                    detection.center_2d.y = float(cy)
                    
                    # Estimate 3D position
                    pos_3d = self.estimate_3d_position([cx, cy], max(w, h))
                    if pos_3d is not None:
                        detection.position_3d.x = pos_3d[0]
                        detection.position_3d.y = pos_3d[1]
                        detection.position_3d.z = pos_3d[2]
                    
                    detections.append(detection)
        
        return detections, mask
        
    def detect_aruco_markers(self, image):
        """Detect ArUco markers for more robust tracking"""
        if not self.use_aruco or self.camera_matrix is None:
            return []
            
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, 
                                                 parameters=self.aruco_params)
        
        detections = []
        if ids is not None:
            # Estimate pose of markers
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, 0.05, self.camera_matrix, self.dist_coeffs)
            
            for i, marker_id in enumerate(ids.flatten()):
                detection = ObjectDetection()
                detection.object_id = f"aruco_{marker_id}"
                detection.confidence = 0.9
                
                # Calculate center from corners
                corner_points = corners[i][0]
                center_x = np.mean(corner_points[:, 0])
                center_y = np.mean(corner_points[:, 1])
                
                detection.center_2d.x = float(center_x)
                detection.center_2d.y = float(center_y)
                
                # 3D position from pose estimation
                detection.position_3d.x = float(tvecs[i][0][0])
                detection.position_3d.y = float(tvecs[i][0][1])
                detection.position_3d.z = float(tvecs[i][0][2])
                
                # Orientation
                rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                quat = Rotation.from_matrix(rotation_matrix).as_quat()
                detection.pose_3d.orientation.x = quat[0]
                detection.pose_3d.orientation.y = quat[1]
                detection.pose_3d.orientation.z = quat[2]
                detection.pose_3d.orientation.w = quat[3]
                
                # Bounding box
                x_coords = corner_points[:, 0]
                y_coords = corner_points[:, 1]
                x, y, w, h = (int(min(x_coords)), int(min(y_coords)),
                             int(max(x_coords) - min(x_coords)),
                             int(max(y_coords) - min(y_coords)))
                detection.bounding_box = [x, y, w, h]
                
                detections.append(detection)
                
        return detections
        
    def estimate_3d_position(self, pixel_coords, object_size_pixels):
        """Estimate 3D position from 2D pixel coordinates"""
        if self.camera_matrix is None:
            return None
            
        # Assume object size of 5cm
        real_object_size = 0.05  # meters
        focal_length = self.camera_matrix[0, 0]
        
        # Estimate depth using similar triangles
        estimated_depth = (real_object_size * focal_length) / object_size_pixels
        
        # Convert to 3D coordinates in camera frame
        u, v = pixel_coords
        x_cam = (u - self.camera_matrix[0, 2]) * estimated_depth / self.camera_matrix[0, 0]
        y_cam = (v - self.camera_matrix[1, 2]) * estimated_depth / self.camera_matrix[1, 1]
        z_cam = estimated_depth
        
        return [x_cam, y_cam, z_cam]
        
    def track_objects(self, detections):
        """Track objects across frames using Kalman filters"""
        tracked_objects = []
        
        for detection in detections:
            # Find closest existing tracker or create new one
            min_distance = float('inf')
            best_tracker_id = None
            
            for tracker_id, kf in self.kalman_filters.items():
                # Predict next position
                prediction = kf.predict()
                predicted_pos = [prediction[0], prediction[1]]
                current_pos = [detection.center_2d.x, detection.center_2d.y]
                
                distance = np.linalg.norm(np.array(predicted_pos) - np.array(current_pos))
                
                if distance < min_distance and distance < 100:  # 100 pixel threshold
                    min_distance = distance
                    best_tracker_id = tracker_id
                    
            if best_tracker_id is not None:
                # Update existing tracker
                kf = self.kalman_filters[best_tracker_id]
                measurement = np.array([[detection.center_2d.x], [detection.center_2d.y]], np.float32)
                kf.correct(measurement)
                detection.object_id += f"_tracked_{best_tracker_id}"
            else:
                # Create new tracker
                kf = self.create_kalman_filter()
                # Initialize state
                kf.statePre = np.array([detection.center_2d.x, detection.center_2d.y, 0, 0], np.float32)
                kf.statePost = np.array([detection.center_2d.x, detection.center_2d.y, 0, 0], np.float32)
                
                self.kalman_filters[self.tracking_id_counter] = kf
                detection.object_id += f"_tracked_{self.tracking_id_counter}"
                self.tracking_id_counter += 1
                
            tracked_objects.append(detection)
            
        return tracked_objects
        
    def draw_detections(self, image, detections, mask=None):
        """Draw detection results on image"""
        debug_image = image.copy()
        
        # Draw mask overlay if provided
        if mask is not None:
            mask_colored = cv2.applyColorMap(mask, cv2.COLORMAP_JET)
            debug_image = cv2.addWeighted(debug_image, 0.7, mask_colored, 0.3, 0)
        
        for detection in detections:
            # Draw bounding box
            if detection.bounding_box:
                x, y, w, h = detection.bounding_box
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
            # Draw center point
            center_x = int(detection.center_2d.x)
            center_y = int(detection.center_2d.y)
            cv2.circle(debug_image, (center_x, center_y), 5, (255, 0, 0), -1)
            
            # Draw ID and confidence
            label = f"{detection.object_id}: {detection.confidence:.2f}"
            cv2.putText(debug_image, label, (center_x - 50, center_y - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Draw 3D position if available
            if detection.position_3d.z != 0:
                pos_text = f"3D: ({detection.position_3d.x:.2f}, {detection.position_3d.y:.2f}, {detection.position_3d.z:.2f})"
                cv2.putText(debug_image, pos_text, (center_x - 50, center_y + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                
        return debug_image
        
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect objects using multiple methods
            color_detections, mask = self.detect_colored_objects(cv_image)
            aruco_detections = self.detect_aruco_markers(cv_image)
            
            # Combine detections
            all_detections = color_detections + aruco_detections
            
            # Track objects
            tracked_detections = self.track_objects(all_detections)
            
            # Update latest detections
            self.latest_detections = tracked_detections
            
            # Create debug image
            debug_image = self.draw_detections(cv_image, tracked_detections, mask)
            
            # Add timestamp and frame info
            timestamp = f"Time: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
            cv2.putText(debug_image, timestamp, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            detection_count = f"Detections: {len(tracked_detections)}"
            cv2.putText(debug_image, detection_count, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f"Vision processing error: {e}")
            
    def get_latest_detections(self):
        """Get latest object detections"""
        return self.latest_detections

def main(args=None):
    rclpy.init(args=args)
    processor = VisionProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()