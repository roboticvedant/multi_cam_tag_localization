#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from std_msgs.msg import Header
from tagpose_interfaces.msg import TagArray, TagPosition
from cv_bridge import CvBridge
import cv2
import numpy as np
from pupil_apriltags import Detector
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation

class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('tag_detect')

        # Initialize member variables
        self._publishers_dict = {}
        self._bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

        # Declare parameters
        self.declare_parameter('camera_name', 'camera_1')
        self.declare_parameter('tag_size', 0.100)
        self.declare_parameter('tag_family', 'tag25h9')
        self.declare_parameter('get_debug_image', True)
        self.declare_parameter('tf2_frame', 'cam1')
        self.declare_parameter('expected_tags', [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34])

        # Get parameters
        self.camera_name = self.get_parameter('camera_name').value
        self.tag_size = float(self.get_parameter('tag_size').value)
        self.tag_family = self.get_parameter('tag_family').value
        self.get_debug_image = self.get_parameter('get_debug_image').value
        self.tf2_frame = self.get_parameter('tf2_frame').value
        self.expected_tags = self.get_parameter('expected_tags').value

        # Initialize detector
        self._detector = Detector(
            families=self.tag_family,
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.5
        )

        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create publisher for TagArray messages
        self.tag_array_pub = self.create_publisher(TagArray, f'/{self.camera_name}/tag_detections', 10)

        # Create publisher for debug image
        if self.get_debug_image:
            self.debug_image_pub = self.create_publisher(
                Image,
                f'/{self.camera_name}/tag_detections/image',
                10
            )

        # Subscribe to image and camera info topics
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

        self.get_logger().info(f'AprilTag detector initialized for {self.camera_name}')

    def _draw_axis(self, img, corners, imgpts):
        """Draw 3D coordinate axis on tag"""
        corner = tuple(corners[0].astype(int))
        img = cv2.line(img, corner, tuple(imgpts[0].ravel().astype(int)), (255,0,0), 3)  # X axis - Red
        img = cv2.line(img, corner, tuple(imgpts[1].ravel().astype(int)), (0,255,0), 3)  # Y axis - Green
        img = cv2.line(img, corner, tuple(imgpts[2].ravel().astype(int)), (0,0,255), 3)  # Z axis - Blue
        return img

    def _create_pose_stamped(self, position, orientation):
        """Create a PoseStamped message with explicit type conversion"""
        pose_stamped = PoseStamped()
        # pose_stamped.header.stamp = self.get_clock().now().to_msg()
        # pose_stamped.header.frame_id = self.tf2_frame
        
        # Ensure position values are float
        pose_stamped.pose.position.x = float(position[0])
        pose_stamped.pose.position.y = float(position[1])
        pose_stamped.pose.position.z = float(position[2])
        
        # Ensure orientation values are float
        pose_stamped.pose.orientation.w = float(orientation[0])
        pose_stamped.pose.orientation.x = float(orientation[1])
        pose_stamped.pose.orientation.y = float(orientation[2])
        pose_stamped.pose.orientation.z = float(orientation[3])
        
        return pose_stamped

    def _camera_info_callback(self, msg):
        """Callback for camera intrinsic calibration info"""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info("Camera calibration loaded from camera_info topic")
            self.destroy_subscription(self._camera_info_sub)

    def _process_tag_detection(self, detection):
        """Process a single tag detection"""
        try:
            # Convert rotation matrix to quaternion using scipy
            R = np.array(detection.pose_R, dtype=np.float64).reshape((3, 3))
            
            # Create 180-degree rotation around X axis
            R_flip = np.array([
                [1,  0,  0],
                [0, -1,  0],
                [0,  0, -1]
            ])
            
            # Apply the flip to the detected rotation
            R_corrected = R @ R_flip
            
            # Convert corrected rotation to quaternion
            rotation = Rotation.from_matrix(R_corrected)
            quat = rotation.as_quat()  # Returns in xyzw format
            
            # Convert to ROS coordinate system (wxyz)
            orientation = [float(quat[3]), float(quat[0]), float(quat[1]), float(quat[2])]
            
            # Get position and ensure it's float
            position = [
                float(detection.pose_t[0]),
                float(detection.pose_t[1]),
                float(detection.pose_t[2])
            ]
            
            # Create pose stamped message
            pose_stamped = self._create_pose_stamped(position, orientation)
            
            # Transform to map frame
            transformed_pose = self._transform_pose_to_map(pose_stamped)

            if transformed_pose is None:
                return None
                
            # Create and return tag position message
            tag_position = TagPosition()
            tag_position.header = pose_stamped.header
            tag_position.family = self.tag_family
            tag_position.id = int(detection.tag_id)
            tag_position.pose = transformed_pose
            tag_position.decision_margin = detection.decision_margin
            tag_position.pose_error = detection.pose_err

            return tag_position
            
        except Exception as e:
            self.get_logger().error(f'Error processing tag detection: {str(e)}')
            return None

    def _transform_pose_to_map(self, pose_stamped):
        """Transform pose from camera frame to map frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.tf2_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            return tf2_geometry_msgs.do_transform_pose(pose_stamped.pose, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Failed to transform pose: {str(e)}')
            self.get_logger().info(f'Available frames: {self.tf_buffer.all_frames_as_string()}')

            return None

    def _image_callback(self, msg):
        """Process image and detect AprilTags"""
        if not self.camera_info_received:
            return

        try:
            detectiontime = msg.header.stamp
            # Convert ROS Image message to OpenCV image
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

            # Create TagArray message
            tag_array_msg = TagArray()
            tag_array_msg.header = Header()
            tag_array_msg.header.stamp = detectiontime
            tag_array_msg.header.frame_id = 'map'
            if tag_array_msg is None:
                self.get_logger().error('Failed to create TagArray message')
                return

            # Create debug image if enabled
            debug_image = cv_image.copy() if self.get_debug_image else None
            
            for detection in detections:
                # Verify detection has pose data
                if not hasattr(detection, 'pose_t') or not hasattr(detection, 'pose_R'):
                    continue

                # Filter out tags that are not expected
                if detection.tag_id not in self.expected_tags:
                    continue

                tag_position = self._process_tag_detection(detection)

                if tag_position is not None:
                    tag_position.header = tag_array_msg.header
                    tag_array_msg.positions.append(tag_position)

                # Draw detection on debug image
                if self.get_debug_image:
                    # Draw tag outline
                    cv2.polylines(debug_image, [np.array(detection.corners, dtype=np.int32)],
                                True, (0, 255, 0), 2)
                    cv2.putText(debug_image, str(detection.tag_id),
                              tuple(detection.corners[0].astype(int)),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    
                    # Project 3D axes
                    axis_length = self.tag_size
                    axis_points = np.float32([[axis_length, 0, 0], 
                                           [0, axis_length, 0], 
                                           [0, 0, axis_length]]).reshape(-1, 3)
                    
                    # Project 3D points to image plane
                    imgpts, _ = cv2.projectPoints(axis_points, 
                                                detection.pose_R, 
                                                detection.pose_t, 
                                                self.camera_matrix, 
                                                self.dist_coeffs)
                    
                    # Draw the axes
                    debug_image = self._draw_axis(debug_image, detection.corners, imgpts)

            # Publish TagArray message
            self.tag_array_pub.publish(tag_array_msg)

            # Publish debug image if enabled
            if self.get_debug_image:
                debug_msg = self._bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AprilTagDetectorNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()