#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from tagpose_interfaces.msg import TagArray, TagPosition
from std_msgs.msg import Header
import message_filters
import numpy as np
from transforms3d.quaternions import mat2quat, quat2mat, qmult, qinverse
import networkx as nx
from typing import Dict, Optional, List, Tuple
from collections import defaultdict

class MultiDetectAggNode(Node):
    def __init__(self):
        super().__init__('multi_detect_agg')

        # Parameters
        self.declare_parameter('cameras', ['camera_1'])
        self.declare_parameter('sync_slop', 0.01)  # Time tolerance for synchronization in seconds
        self.declare_parameter('queue_size', 10)   # Queue size for synchronizer

        # Get parameters
        self.cameras = self.get_parameter('cameras').value
        self.sync_slop = self.get_parameter('sync_slop').value
        self.queue_size = self.get_parameter('queue_size').value

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

    def publish_tag_transform(self, tag_id: int, pose: Pose, camera_names: List[str]):
        """Publish transform for a tag detection"""
        # Create and broadcast transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = f'tag_{tag_id}'

        # Set translation
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z

        # Set rotation
        transform.transform.rotation = pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

        # Log the detection
        detection_type = "single" if len(camera_names) == 1 else "averaged"
        self.get_logger().info(
            f'Broadcasting {detection_type} pose for tag {tag_id} from camera(s) '
            f'{", ".join(camera_names)} at position [{pose.position.x:.3f}, '
            f'{pose.position.y:.3f}, {pose.position.z:.3f}]'
        )

    def sync_callback(self, *tag_arrays):
        """Handle synchronized tag detections from all cameras"""
        try:
            # Dictionary to store all detections of each tag
            # Structure: {tag_id: [(pose, weight, camera_name)]}
            tag_detections = defaultdict(list)

            # Process detections from each camera
            for camera_idx, tag_array in enumerate(tag_arrays):
                camera_name = self.cameras[camera_idx]

                for tag_pos in tag_array.positions:
                    tag_id = tag_pos.id
                    # Store detection with default weight 1.0 and camera name
                    tag_detections[tag_id].append((tag_pos.pose, 1.0, camera_name))

            # Process each detected tag
            for tag_id, detections in tag_detections.items():
                num_detections = len(detections)
                
                if num_detections == 0:
                    continue
                
                # If only one camera detected the tag, use that pose directly
                if num_detections == 1:
                    pose, _, camera_name = detections[0]
                    self.publish_tag_transform(tag_id, pose, [camera_name])
                    continue
                
                # Multiple cameras detected the tag, average the poses
                averaged_pose = self.average_poses([(pose, weight) for pose, weight, _ in detections])
                if averaged_pose is not None:
                    camera_names = [camera for _, _, camera in detections]
                    self.publish_tag_transform(tag_id, averaged_pose, camera_names)

        except Exception as e:
            self.get_logger().error(f'Error in sync callback: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

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
        """Average quaternions using weighted averaging"""
        # Normalize weights
        weights = weights / np.sum(weights)

        # Initialize accumulator
        avg = np.zeros(4)

        # Get reference quaternion
        ref = quaternions[0]

        # Ensure all quaternions are on the same hemisphere as the reference
        for i in range(quaternions.shape[0]):
            if np.dot(ref, quaternions[i]) < 0:
                quaternions[i] = -quaternions[i]

        # Weighted average
        avg = np.sum(quaternions * weights[:, np.newaxis], axis=0)

        # Normalize
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