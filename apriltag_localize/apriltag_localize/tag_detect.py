#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from pupil_apriltags import Detector
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation

class SimpleAprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('tag_detect')
        
        # Initialize basic variables
        self._bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # Parameters
        self.declare_parameter('camera_name', 'camera_1')
        self.declare_parameter('tag_size', 0.100)
        self.declare_parameter('tag_family', 'tag25h9')
        self.declare_parameter('tf2_frame', 'cam1')
        self.declare_parameter('expected_tags', [9, 10, 21])
        
        self.camera_name = self.get_parameter('camera_name').value
        self.tag_size = float(self.get_parameter('tag_size').value)
        self.tag_family = self.get_parameter('tag_family').value
        self.tf2_frame = self.get_parameter('tf2_frame').value
        self.expected_tags = self.get_parameter('expected_tags').value
        
        # Initialize detector
        self._detector = Detector(
            families=self.tag_family,
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25
        )
        
        # Initialize tf2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscribe to camera topics
        self._image_sub = self.create_subscription(
            Image,
            f'/{self.camera_name}/image_raw',
            self._image_callback,
            1
        )
        
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            f'/{self.camera_name}/camera_info',
            self._camera_info_callback,
            1
        )
        
        self.get_logger().info('Simple AprilTag detector initialized')

    def _camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info("Camera calibration received")
            self.destroy_subscription(self._camera_info_sub)

    def _broadcast_tag_transform(self, detection):
        """Broadcast the tag transform directly"""
        try:
            # Create transform message
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.tf2_frame
            t.child_frame_id = f'tag_{detection.tag_id}'
            
            # Set translation (using detected position directly)
            t.transform.translation.x = float(detection.pose_t[0])
            t.transform.translation.y = float(detection.pose_t[1])
            t.transform.translation.z = float(detection.pose_t[2])
            
            # Convert rotation matrix to quaternion
            R = np.array(detection.pose_R, dtype=np.float64).reshape((3, 3))
            rotation = Rotation.from_matrix(R)
            quat = rotation.as_quat()  # xyzw format
            
            # Set rotation
            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])
            
            # Broadcast transform
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f'Error broadcasting transform: {str(e)}')

    def _image_callback(self, msg):
        if not self.camera_info_received:
            return

        try:
            # Convert ROS Image to OpenCV
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect AprilTags
            detections = self._detector.detect(
                gray_image,
                estimate_tag_pose=True,
                camera_params=(
                    float(self.camera_matrix[0, 0]),  # fx
                    float(self.camera_matrix[1, 1]),  # fy
                    float(self.camera_matrix[0, 2]),  # cx
                    float(self.camera_matrix[1, 2])   # cy
                ),
                tag_size=self.tag_size
            )
            
            # Process each detection
            for detection in detections:
                if detection.tag_id not in self.expected_tags:
                    continue
                    
                if hasattr(detection, 'pose_t') and hasattr(detection, 'pose_R'):
                    self._broadcast_tag_transform(detection)
                
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SimpleAprilTagDetectorNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()