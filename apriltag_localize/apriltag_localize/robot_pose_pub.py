#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

class RobotStateEstimator(Node):
    def __init__(self):
        super().__init__('robot_state_estimator')
        
        # Load static transforms from YAML
        self.declare_parameter('tag_transforms_file', 'tag_calibration.yaml')
        yaml_file = self.get_parameter('tag_transforms_file').value
        with open(yaml_file, 'r') as f:
            self.static_transforms = yaml.safe_load(f)
        
        self.declare_parameter('max_detection_age', 0.5)
        self.max_detection_age = Duration(seconds=self.get_parameter('max_detection_age').value)
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.tag_subs = []
        for tag_id in self.static_transforms.keys():
            self.tag_subs.append(
                self.create_subscription(
                    PoseStamped,
                    f'tag{tag_id}/pose',
                    lambda msg, tag_id=tag_id: self.tag_callback(msg, tag_id),
                    10
                )
            )
        
        # Publisher for EKF input
        self.pose_pub = self.create_publisher(
            PoseStamped,
            'robot/pose',
            10
        )
        
        self.latest_detections = {}

    def pose_to_transform_matrix(self, pose):
        """Convert PoseStamped to 4x4 transform matrix"""
        rotation = Rotation.from_quat([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ])
        transform = np.eye(4)
        transform[:3, :3] = rotation.as_matrix()
        transform[:3, 3] = [
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        ]
        return transform

    def transform_matrix_to_pose(self, matrix, frame_id):
        """Convert 4x4 transform matrix to PoseStamped"""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        
        rotation = Rotation.from_matrix(matrix[:3, :3])
        quat = rotation.as_quat()
        
        pose.pose.position.x = matrix[0, 3]
        pose.pose.position.y = matrix[1, 3]
        pose.pose.position.z = matrix[2, 3]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose

    def tag_callback(self, msg: PoseStamped, tag_id: str):
        current_time = self.get_clock().now()
        self.latest_detections[tag_id] = {
            'pose': msg,
            'timestamp': current_time
        }
        
        # Remove stale detections
        self.latest_detections = {
            tag_id: data for tag_id, data in self.latest_detections.items()
            if (current_time - data['timestamp']) <= self.max_detection_age
        }
        
        self.update_robot_state()

    def compute_baselink_pose(self):
        """Compute base_link pose from available tag detections"""
        transforms = []
        
        for tag_id, detection in self.latest_detections.items():
            # Get detected tag pose
            T_map_tag = self.pose_to_transform_matrix(detection['pose'])
            
            # Get static transform from tag to base_link from YAML
            tag_transform = self.static_transforms[f'tag_{tag_id}']
            T_tag_base = np.eye(4)
            T_tag_base[:3, :3] = Rotation.from_quat(tag_transform['rotation']).as_matrix()
            T_tag_base[:3, 3] = tag_transform['translation']
            
            # Compute base_link pose from this tag
            T_map_base = T_map_tag @ T_tag_base
            transforms.append(T_map_base)
        
        if not transforms:
            return None
            
        # Average the transforms
        pos_sum = np.zeros(3)
        quats = []
        
        for T in transforms:
            pos_sum += T[:3, 3]
            quats.append(Rotation.from_matrix(T[:3, :3]).as_quat())
        
        avg_pos = pos_sum / len(transforms)
        avg_quat = Rotation.from_quat(quats).mean().as_quat()
        
        # Construct final transform
        T_final = np.eye(4)
        T_final[:3, :3] = Rotation.from_quat(avg_quat).as_matrix()
        T_final[:3, 3] = avg_pos
        
        return self.transform_matrix_to_pose(T_final, 'map')

    def update_robot_state(self):
        if not self.latest_detections:
            return
        
        baselink_pose = self.compute_baselink_pose()
        if baselink_pose:
            self.pose_pub.publish(baselink_pose)


def main():
    rclpy.init()
    node = RobotStateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
