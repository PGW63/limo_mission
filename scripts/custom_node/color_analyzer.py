import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class ColorAnalyzer:
    def __init__(self):
        rospy.init_node('color_analyzer')
        
        self.bridge = CvBridge()
        self.current_image = None
        self.roi_start = None
        self.roi_end = None
        self.selecting = False
        
        # Subscribe to camera topic
        self.sub_image = rospy.Subscriber(
            '/camera/color/image_raw/compressed', 
            CompressedImage, 
            self.image_callback, 
            queue_size=1
        )
        
        # Create window and set mouse callback
        cv2.namedWindow('Color Analyzer')
        cv2.setMouseCallback('Color Analyzer', self.mouse_callback)
        
        rospy.loginfo("Color Analyzer started. Click and drag to select ROI.")
        rospy.loginfo("Press 'q' to quit, 'a' to analyze selected region.")

    def image_callback(self, msg):
        self.current_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.roi_start = (x, y)
            self.selecting = True
            
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.selecting:
                self.roi_end = (x, y)
                
        elif event == cv2.EVENT_LBUTTONUP:
            self.roi_end = (x, y)
            self.selecting = False
            rospy.loginfo(f"ROI selected: {self.roi_start} to {self.roi_end}")

    def analyze_roi(self):
        if self.current_image is None:
            rospy.logwarn("No image received yet.")
            return
            
        if self.roi_start is None or self.roi_end is None:
            rospy.logwarn("Please select a ROI first.")
            return
        
        # Get ROI coordinates
        x1, y1 = self.roi_start
        x2, y2 = self.roi_end
        x1, x2 = min(x1, x2), max(x1, x2)
        y1, y2 = min(y1, y2), max(y1, y2)
        
        if x1 == x2 or y1 == y2:
            rospy.logwarn("ROI too small.")
            return
        
        # Extract ROI
        roi = self.current_image[y1:y2, x1:x2]
        
        # Convert to different color spaces
        roi_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
        roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        roi_hls = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)
        
        # Calculate statistics
        self.print_statistics(roi_rgb, roi_hsv, roi_hls)
        
        # Plot histograms
        self.plot_histograms(roi_rgb, roi_hsv, roi_hls)

    def print_statistics(self, roi_rgb, roi_hsv, roi_hls):
        rospy.loginfo("=" * 60)
        rospy.loginfo("COLOR STATISTICS:")
        rospy.loginfo("-" * 60)
        
        # RGB
        rgb_mean = roi_rgb.mean(axis=(0, 1)).astype(int)
        rgb_min = roi_rgb.min(axis=(0, 1))
        rgb_max = roi_rgb.max(axis=(0, 1))
        rospy.loginfo(f"RGB - Mean: {rgb_mean}, Min: {rgb_min}, Max: {rgb_max}")
        
        # HSV
        hsv_mean = roi_hsv.mean(axis=(0, 1)).astype(int)
        hsv_min = roi_hsv.min(axis=(0, 1))
        hsv_max = roi_hsv.max(axis=(0, 1))
        rospy.loginfo(f"HSV - Mean: {hsv_mean}, Min: {hsv_min}, Max: {hsv_max}")
        
        # HLS
        hls_mean = roi_hls.mean(axis=(0, 1)).astype(int)
        hls_min = roi_hls.min(axis=(0, 1))
        hls_max = roi_hls.max(axis=(0, 1))
        rospy.loginfo(f"HLS - Mean: {hls_mean}, Min: {hls_min}, Max: {hls_max}")
        rospy.loginfo("=" * 60)

    def plot_histograms(self, roi_rgb, roi_hsv, roi_hls):
        fig, axes = plt.subplots(3, 3, figsize=(15, 12))
        fig.suptitle('Color Space Analysis', fontsize=16)
        
        # RGB Histograms
        colors = ['red', 'green', 'blue']
        labels = ['R', 'G', 'B']
        for i, (color, label) in enumerate(zip(colors, labels)):
            axes[0, i].hist(roi_rgb[:, :, i].ravel(), bins=256, 
                           range=(0, 256), color=color, alpha=0.7)
            axes[0, i].set_title(f'RGB - {label} Channel')
            axes[0, i].set_xlabel('Value')
            axes[0, i].set_ylabel('Frequency')
            axes[0, i].grid(True, alpha=0.3)
        
        # HSV Histograms
        hsv_colors = ['red', 'green', 'blue']
        hsv_labels = ['H', 'S', 'V']
        hsv_ranges = [(0, 180), (0, 256), (0, 256)]
        for i, (color, label, range_val) in enumerate(zip(hsv_colors, hsv_labels, hsv_ranges)):
            axes[1, i].hist(roi_hsv[:, :, i].ravel(), bins=range_val[1], 
                           range=range_val, color=color, alpha=0.7)
            axes[1, i].set_title(f'HSV - {label} Channel')
            axes[1, i].set_xlabel('Value')
            axes[1, i].set_ylabel('Frequency')
            axes[1, i].grid(True, alpha=0.3)
        
        # HLS Histograms
        hls_colors = ['red', 'green', 'blue']
        hls_labels = ['H', 'L', 'S']
        hls_ranges = [(0, 180), (0, 256), (0, 256)]
        for i, (color, label, range_val) in enumerate(zip(hls_colors, hls_labels, hls_ranges)):
            axes[2, i].hist(roi_hls[:, :, i].ravel(), bins=range_val[1], 
                           range=range_val, color=color, alpha=0.7)
            axes[2, i].set_title(f'HLS - {label} Channel')
            axes[2, i].set_xlabel('Value')
            axes[2, i].set_ylabel('Frequency')
            axes[2, i].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()

    def run(self):
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            if self.current_image is not None:
                display_image = self.current_image.copy()
                
                # Draw ROI rectangle if selected
                if self.roi_start is not None and self.roi_end is not None:
                    cv2.rectangle(display_image, self.roi_start, self.roi_end, 
                                (0, 255, 0), 2)
                elif self.roi_start is not None and self.selecting:
                    # Draw temporary rectangle while selecting
                    if self.roi_end is not None:
                        cv2.rectangle(display_image, self.roi_start, self.roi_end, 
                                    (0, 255, 255), 2)
                
                # Add instructions
                cv2.putText(display_image, "Click and drag to select ROI", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(display_image, "Press 'a' to analyze, 'q' to quit", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                cv2.imshow('Color Analyzer', display_image)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('a'):
                self.analyze_roi()
            
            rate.sleep()
        
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        analyzer = ColorAnalyzer()
        analyzer.run()
    except rospy.ROSInterruptException:
        pass