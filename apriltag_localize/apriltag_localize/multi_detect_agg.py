#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from tagpose_interfaces.msg import TagArray, TagPosition
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import message_filters
import numpy as np
from transforms3d.quaternions import mat2quat, quat2mat, qmult, qinverse
from typing import Dict, Optional, List, Tuple
from collections import defaultdict

class MultiDetectAggNode(Node):
    def __init__(self):
        super().__init__('multi_detect_agg')

        # Parameters
        self.declare_parameter('cameras', ['camera_1'])
        self.declare_parameter('sync_slop', 0.01)  # Time tolerance for synchronization in seconds
        self.declare_parameter('queue_size', 5)   # Queue size for synchronizer
        self.declare_parameter('debug_cam', True)

        # Get parameters
        self.cameras = self.get_parameter('cameras').value
        self.sync_slop = self.get_parameter('sync_slop').value
        self.queue_size = self.get_parameter('queue_size').value
        self.debug_cam = self.get_parameter('debug_cam').value

        # Initialize tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create synchronized subscribers for each camera
        self.tag_subs = []
        for camera in self.cameras:
            sub = message_filters.Subscriber(
                self,
                TagArray,
                f'/{camera}/tag_detections'
            )
            self.tag_subs.append(sub)

        # Create approximate time synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            self.tag_subs,
            queue_size=self.queue_size,
            slop=self.sync_slop
        )
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info(
            f'Multi-detect aggregator initialized for cameras: {self.cameras}\n'
            f'Sync slop: {self.sync_slop} seconds, Queue size: {self.queue_size}'
        )

    def nanosec_to_time_msg(self, total_nanosec: int) -> Time:
        """Convert total nanoseconds to ROS Time message"""
        seconds = total_nanosec // 1_000_000_000
        nanoseconds = total_nanosec % 1_000_000_000
        time_msg = Time()
        time_msg.sec = int(seconds)
        time_msg.nanosec = int(nanoseconds)
        return time_msg

    def create_transform_stamped(self, tag_id: int, pose: Pose, frame_id: str, timestamp: Time, child_frame_suffix: str = '') -> TransformStamped:
        """Create a TransformStamped message"""
        transform = TransformStamped()
        transform.header.stamp = timestamp  # Already a Time message
        transform.header.frame_id = frame_id
        transform.child_frame_id = f'tag_{tag_id}{child_frame_suffix}'

        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation

        return transform

    def publish_tag_transforms(self, tag_detections_by_camera: Dict[str, List[Tuple[int, Pose, float]]], 
                            timestamps: Dict[str, Time]):
        """Publish all transforms for each camera's detections together"""
        transforms_to_publish = []

        # First collect all detections by tag_id
        tags_with_detections = defaultdict(list)
        for camera, detections in tag_detections_by_camera.items():
            for tag_id, pose, decision_margin in detections:
                tags_with_detections[tag_id].append((pose, decision_margin, camera))
                
                # If debug is enabled, always publish the camera-specific transform
                if self.debug_cam:
                    transform_debug = self.create_transform_stamped(
                        tag_id=tag_id,
                        pose=pose,
                        frame_id='map',
                        timestamp=timestamps[camera],
                        child_frame_suffix=f'_{camera}'
                    )
                    transforms_to_publish.append(transform_debug)

        # Now handle the main transforms (without camera suffix)
        for tag_id, detections in tags_with_detections.items():
            if len(detections) > 1:
                # Multiple cameras saw this tag → Compute averaged pose
                cameras_seeing_tag = [camera for _, _, camera in detections]
                
                # Get latest timestamp
                total_nanoseconds = [
                    timestamps[camera].sec * 1_000_000_000 + timestamps[camera].nanosec 
                    for camera in cameras_seeing_tag
                ]
                latest_timestamp = self.nanosec_to_time_msg(max(total_nanoseconds))
                
                # Compute weighted average pose
                total_margin = sum([margin for _, margin, _ in detections])
                poses_with_weights = [(pose, margin / total_margin) for pose, margin, _ in detections]

                averaged_pose = self.average_poses(poses_with_weights)
                if averaged_pose is not None:
                    # Publish the averaged transform without camera suffix
                    transform = self.create_transform_stamped(
                        tag_id=tag_id,
                        pose=averaged_pose,
                        frame_id='map',
                        timestamp=latest_timestamp
                    )
                    transforms_to_publish.append(transform)
            else:
                # Single camera detection - publish without suffix
                pose, margin, camera = detections[0]
                transform = self.create_transform_stamped(
                    tag_id=tag_id,
                    pose=pose,
                    frame_id='map',
                    timestamp=timestamps[camera]
                )
                transforms_to_publish.append(transform)

        # Publish all transforms together
        if transforms_to_publish:
            self.tf_broadcaster.sendTransform(transforms_to_publish)
            self.log_transforms(transforms_to_publish)




    def log_transforms(self, transforms: List[TransformStamped]):
        """Log information about published transforms"""
        for transform in transforms:
            timestamp_sec = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            self.get_logger().info(
                f'Published transform for {transform.child_frame_id} '
                f'at timestamp {timestamp_sec:.3f}s '
                f'position: [{transform.transform.translation.x:.3f}, '
                f'{transform.transform.translation.y:.3f}, '
                f'{transform.transform.translation.z:.3f}]'
            )

    def sync_callback(self, *tag_arrays):
        """Handle synchronized tag detections from all cameras"""
        try:
            # Dictionary to store detections by camera
            tag_detections_by_camera = {}
            timestamps = {}

            # Process detections from each camera
            for camera_idx, tag_array in enumerate(tag_arrays):
                camera_name = self.cameras[camera_idx]
                
                # Store the timestamp for this camera's detections
                timestamps[camera_name] = tag_array.header.stamp
                
                # Store all detections from this camera
                camera_detections = []
                for tag_pos in tag_array.positions:
                    camera_detections.append((
                        tag_pos.id,
                        tag_pos.pose.pose.pose,
                        tag_pos.decision_margin
                    ))
                
                if camera_detections:  # Only store if there were any detections
                    tag_detections_by_camera[camera_name] = camera_detections

            # Publish all transforms together if there are any detections
            if tag_detections_by_camera:
                self.publish_tag_transforms(tag_detections_by_camera, timestamps)

        except Exception as e:
            self.get_logger().error(f'Error in sync callback: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def average_poses(self, pose_weights: List[Tuple[Pose, float]]) -> Optional[Pose]:
        """Average multiple poses with weights"""
        if not pose_weights:
            return None

        total_weight = 0.0
        weighted_position = np.zeros(3)
        quats = []
        weights = []

        for pose, weight in pose_weights:
            total_weight += weight
            weights.append(weight)

            # Accumulate weighted positions
            weighted_position += weight * np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z
            ])

            # Collect quaternions for averaging
            quats.append([
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            ])

        if total_weight == 0:
            return None

        # Normalize weighted position
        avg_position = weighted_position / total_weight

        # Average quaternions with weights
        avg_quat = self.average_quaternions(np.array(quats), np.array(weights))

        # Create averaged pose
        averaged_pose = Pose()
        averaged_pose.position.x = float(avg_position[0])
        averaged_pose.position.y = float(avg_position[1])
        averaged_pose.position.z = float(avg_position[2])
        averaged_pose.orientation.w = float(avg_quat[0])
        averaged_pose.orientation.x = float(avg_quat[1])
        averaged_pose.orientation.y = float(avg_quat[2])
        averaged_pose.orientation.z = float(avg_quat[3])

        return averaged_pose

    def average_quaternions(self, quaternions: np.ndarray, weights: np.ndarray) -> np.ndarray:
        """Simple weighted average of quaternions with normalization"""
        # Simple weighted average since quaternions should be very close
        avg = np.sum(quaternions * weights[:, np.newaxis], axis=0)
        
        # Normalize to unit quaternion
        return avg / np.linalg.norm(avg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MultiDetectAggNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()