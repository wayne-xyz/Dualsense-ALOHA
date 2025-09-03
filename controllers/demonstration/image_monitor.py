"""
Image monitoring utilities for ACT inference visualization.

This module provides real-time camera image display in a 2x2 grid layout,
extracted from the main inference code to keep it clean and modular.

Usage:
    from image_monitor import ImageMonitor
    
    monitor = ImageMonitor(camera_names=['cam1', 'cam2', 'cam3', 'cam4'])
    monitor.create_monitoring_windows()
    monitor.update_image_monitoring(image_list)
    monitor.close_monitoring_windows()
"""

import cv2
import numpy as np
from typing import List, Optional


class ImageMonitor:
    """Manages real-time image monitoring windows for multiple camera feeds."""
    
    def __init__(self, camera_names: List[str], enabled: bool = True):
        """
        Initialize image monitor.
        
        Args:
            camera_names: List of camera names to monitor
            enabled: Whether monitoring is enabled
        """
        self.camera_names = camera_names
        self.enabled = enabled
        self.monitor_window_name = "Camera Monitor - All Views"
        self.monitor_windows_created = False
        
    def create_monitoring_windows(self):
        """Create single OpenCV window with 2x2 grid for monitoring camera images"""
        if not self.enabled:
            return
            
        # Create single window for all camera views
        cv2.namedWindow(self.monitor_window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.monitor_window_name, 640, 480)  # 2x2 grid of 320x240 images
        
        self.monitor_windows_created = True
        print("Created unified image monitoring window")
    
    def update_image_monitoring(self, images_list: List[np.ndarray]):
        """
        Update single monitoring window with 2x2 grid of current images.
        
        Args:
            images_list: List of images corresponding to camera_names
        """
        if not self.enabled or not self.monitor_windows_created:
            return
        
        # Process all images and create composite
        processed_images = []
        
        for i, cam_name in enumerate(self.camera_names):
            if i < len(images_list):
                img = images_list[i]
                
                # Convert from policy format (C, H, W) to display format (H, W, C)
                if len(img.shape) == 3 and img.shape[0] == 3:
                    display_img = np.transpose(img, (1, 2, 0))
                else:
                    display_img = img
                
                # Convert to uint8 if needed
                if display_img.dtype == np.float32 or display_img.dtype == np.float64:
                    display_img = (display_img * 255).astype(np.uint8)
                
                # Convert RGB to BGR for OpenCV display
                if len(display_img.shape) == 3:
                    display_img = cv2.cvtColor(display_img, cv2.COLOR_RGB2BGR)
                
                # Resize to standard monitoring size
                display_img = cv2.resize(display_img, (320, 240))
                
                # Add camera name label
                cv2.putText(display_img, cam_name, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                processed_images.append(display_img)
            else:
                # Create black placeholder if camera missing
                placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(placeholder, f"No {cam_name}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                processed_images.append(placeholder)
        
        # Ensure we have exactly 4 images (pad with black if needed)
        while len(processed_images) < 4:
            placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(placeholder, "Empty", (120, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
            processed_images.append(placeholder)
        
        # Create 2x2 grid composite image
        # Top row: cameras 0 and 1
        top_row = np.hstack([processed_images[0], processed_images[1]])
        # Bottom row: cameras 2 and 3  
        bottom_row = np.hstack([processed_images[2], processed_images[3]])
        # Combine rows
        composite_image = np.vstack([top_row, bottom_row])
        
        # Display composite image
        cv2.imshow(self.monitor_window_name, composite_image)
        
        # Non-blocking waitKey to refresh window
        cv2.waitKey(1)
    
    def close_monitoring_windows(self):
        """Close all monitoring windows"""
        if self.enabled and self.monitor_windows_created:
            cv2.destroyAllWindows()
            print("Closed image monitoring windows")