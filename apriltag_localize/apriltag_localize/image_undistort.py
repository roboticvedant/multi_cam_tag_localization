#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageUndistorter(Node):
    def __init__(self):
        super().__init__('image_undistorter')
        
        # Parameters
        self.declare_parameter('display_images', False)
        self.display_images = self.get_parameter('display_images').value
        
        # Subscriptions with QoS profiles
        self.image_sub = self.create_subscription(
            Image, 
            'image_raw', 
            self.image_callback, 
            1
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 
            'camera_info', 
            self.camera_info_callback, 
            1
        )
        
        # Publisher
        self.undistorted_image_pub = self.create_publisher(
            Image, 
            'image_undistorted', 
            1
        )
        
        # OpenCV Bridge
        self.bridge = CvBridge()
        
        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None
        self.roi = None
        
        self.get_logger().info('Image Undistorter node initialized')

    def camera_info_callback(self, msg):
        try:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            
            # Calculate optimal camera matrix
            image_size = (msg.width, msg.height)
            self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, 
                self.dist_coeffs, 
                image_size, 
                1, 
                image_size
            )
            
            # self.get_logger().info('Camera parameters updated successfully')
        except Exception as e:
            self.get_logger().error(f'Error processing camera info: {str(e)}')

    def image_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn('Waiting for camera parameters...')
            return

        try:
            # Convert ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Undistort the image
            undistorted_frame = cv2.undistort(
                frame, 
                self.camera_matrix, 
                self.dist_coeffs, 
                None,
                self.new_camera_matrix
            )
            
            # Crop the image based on ROI if needed
            if self.roi:
                x, y, w, h = self.roi
                undistorted_frame = undistorted_frame[y:y+h, x:x+w]
            
            # Convert back to ROS message and publish
            undistorted_msg = self.bridge.cv2_to_imgmsg(undistorted_frame, encoding='bgr8')
            undistorted_msg.header = msg.header  # Preserve timestamp and frame_id
            self.undistorted_image_pub.publish(undistorted_msg)
            
            # Display images if enabled
            if self.display_images:
                cv2.imshow("Original Image", frame)
                cv2.imshow("Undistorted Image", undistorted_frame)
                cv2.waitKey(1)
                
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImageUndistorter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Unexpected error: {str(e)}')
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()