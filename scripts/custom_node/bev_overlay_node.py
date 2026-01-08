#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BEV Overlay Node
- Subscribes to BEV visualization from lane detection
- Subscribes to original camera image
- Inverse warps BEV back to original perspective
- Overlays on original image with transparency
- Publishes composite image
"""

import rospy 
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class BEVOverlayNode:
    def __init__(self):
        rospy.init_node('bev_overlay_node')
        self.bridge = CvBridge()
        
        # Image parameters
        self.img_width = 640
        self.img_height = 480
        
        # BEV transformation parameters (must match lane detection node)
        self.src_points = np.float32([
            [80.0, 310.0],   # Top-left
            [560.0, 310.0],   # Top-right
            [640, 480],   # Bottom-right
            [0, 480]      # Bottom-left
        ])
        
        self.dst_points = np.float32([
            [120, 0],          # Top-left
            [520, 0],          # Top-right
            [520, 480],        # Bottom-right
            [120, 480]         # Bottom-left
        ])
        
        # Compute inverse perspective transform matrix (BEV -> original)
        self.M_inv = cv2.getPerspectiveTransform(self.dst_points, self.src_points)
        
        # Overlay parameters
        self.overlay_alpha = 0.6  # Transparency for BEV overlay (0.0 = transparent, 1.0 = opaque)
        
        # Storage for latest images
        self.latest_camera_image = None
        self.latest_bev_image = None
        
        # Subscribers
        self.camera_sub = rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, 
                                          self.camera_callback, queue_size=1, buff_size=2**24)
        self.bev_sub = rospy.Subscriber('/lane_detection/bev/compressed', CompressedImage, 
                                       self.bev_callback, queue_size=1, buff_size=2**24)
        
        # Publisher
        self.overlay_pub = rospy.Publisher('/lane_detection/overlay/compressed', CompressedImage, queue_size=1)
        
        rospy.loginfo("="*50)
        rospy.loginfo("BEV Overlay Node Initialized")
        rospy.loginfo("Subscribing to:")
        rospy.loginfo("  - /camera/color/image_raw/compressed")
        rospy.loginfo("  - /lane_detection/bev/compressed")
        rospy.loginfo("Publishing to:")
        rospy.loginfo("  - /lane_detection/overlay/compressed")
        rospy.loginfo("="*50)
    
    def camera_callback(self, msg):
        """Store latest camera image"""
        try:
            self.latest_camera_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            self.process_overlay()
        except Exception as e:
            rospy.logerr(f"Error in camera_callback: {e}")
    
    def bev_callback(self, msg):
        """Store latest BEV image"""
        try:
            self.latest_bev_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            self.process_overlay()
        except Exception as e:
            rospy.logerr(f"Error in bev_callback: {e}")
    
    def process_overlay(self):
        """Create and publish overlay image"""
        # Check if both images are available
        if self.latest_camera_image is None or self.latest_bev_image is None:
            return
        
        try:
            # Get copies to work with
            camera_img = self.latest_camera_image.copy()
            bev_img = self.latest_bev_image.copy()
            
            # Ensure images are the right size
            if camera_img.shape[:2] != (self.img_height, self.img_width):
                camera_img = cv2.resize(camera_img, (self.img_width, self.img_height))
            if bev_img.shape[:2] != (self.img_height, self.img_width):
                bev_img = cv2.resize(bev_img, (self.img_width, self.img_height))
            
            # Inverse warp BEV back to original perspective
            bev_unwarped = cv2.warpPerspective(bev_img, self.M_inv, 
                                              (self.img_width, self.img_height))
            
            # Create mask for BEV overlay region (non-black pixels)
            gray_bev = cv2.cvtColor(bev_unwarped, cv2.COLOR_BGR2GRAY)
            mask = (gray_bev > 10).astype(np.uint8)  # Threshold to avoid black regions
            
            # Create 3-channel mask
            mask_3ch = cv2.merge([mask, mask, mask])
            
            # Blend images: overlay BEV on camera image
            # result = camera * (1 - alpha * mask) + bev_unwarped * (alpha * mask)
            overlay_result = camera_img.copy()
            overlay_result = cv2.addWeighted(
                camera_img, 1.0,
                np.zeros_like(camera_img), 0.0,
                0.0
            )
            
            # Apply weighted overlay only where BEV has content
            for c in range(3):
                overlay_result[:, :, c] = np.where(
                    mask > 0,
                    camera_img[:, :, c] * (1 - self.overlay_alpha) + 
                    bev_unwarped[:, :, c] * self.overlay_alpha,
                    camera_img[:, :, c]
                )
            
            # Draw trapezoid border to show BEV region
            pts = self.src_points.astype(np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(overlay_result, [pts], True, (255, 255, 0), 2)
            
            # Add text information
            cv2.putText(overlay_result, "BEV Overlay", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(overlay_result, f"Alpha: {self.overlay_alpha:.1f}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Publish overlay image
            overlay_msg = self.bridge.cv2_to_compressed_imgmsg(overlay_result)
            self.overlay_pub.publish(overlay_msg)
            
        except Exception as e:
            rospy.logerr(f"Error in process_overlay: {e}")
    
    def run(self):
        """Run the node"""
        rospy.spin()


def main():
    try:
        node = BEVOverlayNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
