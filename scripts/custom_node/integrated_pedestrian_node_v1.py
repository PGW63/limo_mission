#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class IntegratedLanePedestrianNode:
    def __init__(self):
        rospy.init_node('integrated_lane_pedestrian_node')
        
        self.current_state = None
        self.bridge = CvBridge()
        
        # Lane following parameters
        self.lane_center_x = None
        self.left_lane_x = None
        self.right_lane_x = None
        self.image_width = 640
        self.image_height = 480
        
        # Pedestrian detection parameters (LiDAR based)
        self.pedestrian_detected = False
        self.stop_distance = 0.5  # meters
        self.detection_angle_range = 30  # degrees from center
        self.min_distance = float('inf')
        
        # Control parameters
        self.max_linear_speed = 0.3
        self.max_angular_speed = 1.0
        
        # LiDAR data
        self.scan_data = None
        
        # Subscribers
        self.sub_state = rospy.Subscriber('/state_manager/state', String, self.state_callback, queue_size=1)
        self.sub_image = rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self.image_callback, queue_size=1)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        
        # Publishers
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_lane_debug = rospy.Publisher('/integrated_node/lane_debug/compressed', CompressedImage, queue_size=1)
        self.pub_pedestrian_debug = rospy.Publisher('/integrated_node/pedestrian_debug/compressed', CompressedImage, queue_size=1)
        
        rospy.loginfo("Integrated Lane & Pedestrian Node initialized")

    def state_callback(self, msg: String):
        self.current_state = msg.data
        rospy.loginfo(f"State changed to: {self.current_state}")

    def scan_callback(self, msg: LaserScan):
        """Store LiDAR scan data"""
        self.scan_data = msg

    def image_callback(self, msg: CompressedImage):
        # Only process if in DETECT_PEDESTRIAN state
        if self.current_state != "DETECT_PEDESTRIAN":
            return
        
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        # Process lane detection
        lane_image = cv_image.copy()
        lane_detected = self.detect_lane(lane_image)
        
        # Process pedestrian detection (LiDAR based)
        pedestrian_image = cv_image.copy()
        pedestrian_detected = self.detect_pedestrian_lidar()
        
        # Generate control command
        self.generate_control_command(lane_detected, pedestrian_detected)
        
        # Publish separate debug images
        self.publish_lane_debug_image(lane_image, lane_detected)
        self.publish_pedestrian_debug_image(pedestrian_image, pedestrian_detected)

    def detect_lane(self, image):
        """Detect left and right lanes separately and calculate center"""
        height, width = image.shape[:2]
        
        # ROI for lane detection (bottom half)
        roi_top = int(height * 0.5)
        roi = image[roi_top:height, :]
        
        # Convert to HSV and detect white lane
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # White lane mask
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([180, 30, 255])
        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        
        # Find contours
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Split image into left and right halves
        image_center = width // 2
        
        left_contours = []
        right_contours = []
        
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                
                # Classify contour as left or right based on centroid
                if cx < image_center:
                    left_contours.append(contour)
                else:
                    right_contours.append(contour)
        
        # Find the largest contour on each side
        self.left_lane_x = None
        self.right_lane_x = None
        
        if left_contours:
            largest_left = max(left_contours, key=cv2.contourArea)
            M = cv2.moments(largest_left)
            if M["m00"] > 0:
                self.left_lane_x = int(M["m10"] / M["m00"])
                
                # Draw left lane
                adjusted_contour = largest_left.copy()
                adjusted_contour[:, :, 1] += roi_top
                cv2.drawContours(image, [adjusted_contour], -1, (0, 255, 0), 2)
                cy = roi_top + int(M["m01"] / M["m00"])
                cv2.circle(image, (self.left_lane_x, cy), 10, (0, 255, 0), -1)
        
        if right_contours:
            largest_right = max(right_contours, key=cv2.contourArea)
            M = cv2.moments(largest_right)
            if M["m00"] > 0:
                self.right_lane_x = int(M["m10"] / M["m00"])
                
                # Draw right lane
                adjusted_contour = largest_right.copy()
                adjusted_contour[:, :, 1] += roi_top
                cv2.drawContours(image, [adjusted_contour], -1, (255, 0, 0), 2)
                cy = roi_top + int(M["m01"] / M["m00"])
                cv2.circle(image, (self.right_lane_x, cy), 10, (255, 0, 0), -1)
        
        # Calculate lane center
        if self.left_lane_x is not None and self.right_lane_x is not None:
            # Both lanes detected - use midpoint
            self.lane_center_x = (self.left_lane_x + self.right_lane_x) // 2
            return True
        elif self.left_lane_x is not None:
            # Only left lane detected - assume right lane position
            assumed_right = self.left_lane_x + 300  # Adjust offset as needed
            self.lane_center_x = (self.left_lane_x + assumed_right) // 2
            return True
        elif self.right_lane_x is not None:
            # Only right lane detected - assume left lane position
            assumed_left = self.right_lane_x - 300  # Adjust offset as needed
            self.lane_center_x = (assumed_left + self.right_lane_x) // 2
            return True
        
        self.lane_center_x = None
        return False

    def detect_pedestrian_lidar(self):
        """Detect pedestrian using LiDAR data"""
        if self.scan_data is None:
            return False
        
        # Get center region indices
        num_readings = len(self.scan_data.ranges)
        angle_increment = self.scan_data.angle_increment
        
        # Calculate index range for detection angle
        angle_range_rad = np.deg2rad(self.detection_angle_range)
        index_range = int(angle_range_rad / angle_increment / 2)
        
        center_idx = num_readings // 2
        start_idx = max(0, center_idx - index_range)
        end_idx = min(num_readings, center_idx + index_range)
        
        # Check for obstacles in front
        front_ranges = self.scan_data.ranges[start_idx:end_idx]
        
        # Filter out invalid readings
        valid_ranges = [r for r in front_ranges if not np.isinf(r) and not np.isnan(r) and r > 0]
        
        if valid_ranges:
            self.min_distance = min(valid_ranges)
            
            if self.min_distance < self.stop_distance:
                self.pedestrian_detected = True
                rospy.logwarn(f"Pedestrian detected at {self.min_distance:.2f}m")
                return True
        else:
            self.min_distance = float('inf')
        
        self.pedestrian_detected = False
        return False

    def generate_control_command(self, lane_detected, pedestrian_detected):
        """Generate Twist command based on lane and pedestrian detection"""
        cmd = Twist()
        
        # If pedestrian detected, stop
        if pedestrian_detected:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            rospy.logwarn("Pedestrian detected - STOPPING")
        
        # If no pedestrian and lane detected, follow lane
        elif lane_detected and self.lane_center_x is not None:
            # Calculate error from center
            error = self.lane_center_x - (self.image_width / 2)
            
            # Proportional control
            Kp = 0.003  # Adjust proportional gain
            angular_z = -Kp * error
            
            # Clamp angular velocity
            angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
            
            cmd.linear.x = self.max_linear_speed
            cmd.angular.z = angular_z
            
            rospy.loginfo(f"Lane following - Error: {error:.2f}, Angular: {angular_z:.2f}")
        
        # If no lane detected, stop
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            rospy.logwarn("No lane detected - STOPPING")
        
        self.pub_cmd_vel.publish(cmd)

    def publish_lane_debug_image(self, image, lane_detected):
        """Publish lane detection debug image"""
        # Darken image
        debug_image = (image * 0.6).astype(np.uint8)
        
        # Add title
        cv2.putText(debug_image, "LANE DETECTION", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # Add state
        cv2.putText(debug_image, f"State: {self.current_state}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add lane status
        lane_status = "Detected" if lane_detected else "Not Found"
        lane_color = (0, 255, 0) if lane_detected else (0, 0, 255)
        cv2.putText(debug_image, f"Lane: {lane_status}", (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, lane_color, 2)
        
        # Add left/right lane positions
        if self.left_lane_x is not None:
            cv2.line(debug_image, (self.left_lane_x, 0), 
                    (self.left_lane_x, image.shape[0]), (0, 255, 0), 2)
            cv2.putText(debug_image, f"Left: {self.left_lane_x}", (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        if self.right_lane_x is not None:
            cv2.line(debug_image, (self.right_lane_x, 0), 
                    (self.right_lane_x, image.shape[0]), (255, 0, 0), 2)
            cv2.putText(debug_image, f"Right: {self.right_lane_x}", (10, 150),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # Add lane center indicator
        if self.lane_center_x is not None:
            cv2.line(debug_image, (self.lane_center_x, 0), 
                    (self.lane_center_x, image.shape[0]), (0, 255, 255), 3)
            cv2.putText(debug_image, f"Center: {self.lane_center_x}", (10, 180),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Add image center line
        cv2.line(debug_image, (self.image_width // 2, 0), 
                (self.image_width // 2, image.shape[0]), (255, 255, 255), 1)
        
        # Draw ROI rectangle
        roi_top = int(self.image_height * 0.5)
        cv2.rectangle(debug_image, (0, roi_top), 
                     (self.image_width, self.image_height), (255, 255, 0), 2)
        
        # Publish
        debug_msg = self.bridge.cv2_to_compressed_imgmsg(debug_image, dst_format='jpeg')
        self.pub_lane_debug.publish(debug_msg)

    def publish_pedestrian_debug_image(self, image, pedestrian_detected):
        """Publish pedestrian detection debug image"""
        # Darken image
        debug_image = (image * 0.6).astype(np.uint8)
        
        # Add title
        cv2.putText(debug_image, "PEDESTRIAN DETECTION (LiDAR)", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # Add state
        cv2.putText(debug_image, f"State: {self.current_state}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add pedestrian status
        ped_status = "DETECTED!" if pedestrian_detected else "Clear"
        ped_color = (0, 0, 255) if pedestrian_detected else (0, 255, 0)
        cv2.putText(debug_image, f"Pedestrian: {ped_status}", (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, ped_color, 2)
        
        # Add distance info
        if self.min_distance != float('inf'):
            cv2.putText(debug_image, f"Distance: {self.min_distance:.2f}m", (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(debug_image, f"Threshold: {self.stop_distance:.2f}m", (10, 150),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add LiDAR status
        lidar_status = "Active" if self.scan_data is not None else "No Data"
        lidar_color = (0, 255, 0) if self.scan_data is not None else (0, 0, 255)
        cv2.putText(debug_image, f"LiDAR: {lidar_status}", (10, 180),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, lidar_color, 2)
        
        # Draw detection zone (visualization)
        height, width = debug_image.shape[:2]
        center_x = width // 2
        zone_width = int(width * 0.3)
        cv2.rectangle(debug_image, 
                     (center_x - zone_width // 2, int(height * 0.3)),
                     (center_x + zone_width // 2, int(height * 0.8)),
                     (255, 0, 255), 2)
        cv2.putText(debug_image, "Detection Zone", 
                   (center_x - 80, int(height * 0.25)),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        
        # Publish
        debug_msg = self.bridge.cv2_to_compressed_imgmsg(debug_image, dst_format='jpeg')
        self.pub_pedestrian_debug.publish(debug_msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = IntegratedLanePedestrianNode()
        node.run()
    except rospy.ROSInterruptException:
        pass