#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from geometry_msgs.msg import (PoseStamped, TransformStamped, Transform, 
                             Vector3, Quaternion, Point, PoseWithCovarianceStamped)
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_msgs.msg import TFMessage
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from collections import defaultdict
from sensor_msgs.msg import Imu, MagneticField

class RobotStateEstimator(Node):
    def __init__(self):
        super().__init__('robot_state_estimator')
        
        # Debug parameter
        self.declare_parameter('publish_debug_tf', True)
        self.publish_debug_tf = self.get_parameter('publish_debug_tf').value
        
        # Load static transforms from YAML
        self.declare_parameter('tag_transforms_file', 'tag_calibration.yaml')
        yaml_file = self.get_parameter('tag_transforms_file').value
        with open(yaml_file, 'r') as f:
            self.static_transforms = yaml.safe_load(f)['transforms']
        
        self.declare_parameter('max_detection_age', 0.1)
        self.max_detection_age = Duration(seconds=self.get_parameter('max_detection_age').value)
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store last processed timestamp to avoid duplicate processing
        self.last_processed_stamps = defaultdict(lambda: 0.0)
        
        # TF Subscriber
        self.tf_sub = self.create_subscription(
            TFMessage, 
            '/tf',
            self.tf_callback,
            10
        )

        self.subscription_imu = self.create_subscription(
            Imu,
            '/imu/data',  # Replace with your IMU topic
            self.imu_callback,
            1
        )
        
        # Publisher for EKF input
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'robot/pose',
            10
        )
        
        self.latest_detections = defaultdict(list)
        self.get_logger().info('Robot state estimator node started')

    def transform_to_pose(self, transform: Transform) -> tuple:
        """Convert Transform to position and rotation"""
        pos = np.array([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z
        ])
        rot = Rotation.from_quat([
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        ])
        return pos, rot
    
    def imu_callback(self, msg: Imu):
        self.imu_orrientation = msg.orientation

    def create_transform(self, pos: np.ndarray, rot: Rotation) -> Transform:
        """Create Transform message from position and rotation"""
        transform = Transform()
        transform.translation = Vector3(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        quat = rot.as_quat()
        transform.rotation = Quaternion(x=float(quat[0]), y=float(quat[1]), 
                                     z=float(quat[2]), w=float(quat[3]))
        return transform

    def inverse_transform(self, transform: Transform) -> Transform:
        """Compute inverse of a transform"""
        pos, rot = self.transform_to_pose(transform)
        # Inverse rotation
        rot_inv = rot.inv()
        # Inverse translation
        pos_inv = -(rot_inv.as_matrix() @ pos)
        return self.create_transform(pos_inv, rot_inv)

    def compose_transforms(self, t1: Transform, t2: Transform) -> Transform:
        """Compose two transforms t1 * t2"""
        pos1, rot1 = self.transform_to_pose(t1)
        pos2, rot2 = self.transform_to_pose(t2)
        
        # Compose rotations
        rot_combined = rot1 * rot2
        # Compose translations
        pos_combined = pos1 + rot1.as_matrix() @ pos2
        
        return self.create_transform(pos_combined, rot_combined)

    def pose_to_transform_stamped(self, pose: PoseWithCovarianceStamped) -> TransformStamped:
        """Convert PoseStamped to TransformStamped"""
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = 'base_link_estimated'
        transform.transform.translation = Vector3(x=pose.pose.pose.position.x,
                                               y=pose.pose.pose.position.y,
                                               z=pose.pose.pose.position.z)
        transform.transform.rotation = pose.pose.pose.orientation
        return transform

    def verify_transform_frames(self, transform: TransformStamped) -> bool:
        """Verify transform is from map to tag"""
        if transform.header.frame_id != 'map':
            # self.get_logger().warn(f'Expected map frame, got {transform.header.frame_id}')
            return False
        return True

    def tf_callback(self, msg):
        should_update = False
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        try:
            for transform in msg.transforms:
                if not self.verify_transform_frames(transform):
                    continue
                
                tag_id = transform.child_frame_id
                if tag_id not in self.static_transforms:
                    continue
                
                # Check if this transform is newer than last processed
                transform_time = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                if transform_time <= self.last_processed_stamps[tag_id]:
                    continue
                
                self.last_processed_stamps[tag_id] = transform_time
                self.latest_detections[tag_id].append((current_time, transform))
                should_update = True
                self.get_logger().debug(f"New detection for {tag_id}")
            
            # Clean up stale detections
            timeout = self.max_detection_age.nanoseconds / 1e9
            for tag_id in list(self.latest_detections.keys()):
                fresh_detections = [
                    (t, trans) for t, trans in self.latest_detections[tag_id]
                    if current_time - t < timeout
                ]
                if fresh_detections:
                    self.latest_detections[tag_id] = fresh_detections
                else:
                    del self.latest_detections[tag_id]
            
            # Only update and publish if we received new transforms
            if should_update:
                self.update_robot_state()
                
        except Exception as e:
            self.get_logger().error(f'Error in tf_callback: {str(e)}')


    def predict_occluded_tag(self, visible_tag_id: str, visible_tag_pose: Transform) -> dict:
        """
        Predict poses of other tags using calibration data and one visible tag
        Returns dict of predicted tag poses
        """
        predictions = {}
        
        # Get visible tag's calibration
        visible_tag_calib = self.static_transforms[visible_tag_id]
        T_base_visible = Transform()
        T_base_visible.translation = Vector3(x=visible_tag_calib['translation'][0],
                                          y=visible_tag_calib['translation'][1],
                                          z=visible_tag_calib['translation'][2])
        T_base_visible.rotation = Quaternion(x=visible_tag_calib['rotation'][0],
                                          y=visible_tag_calib['rotation'][1],
                                          z=visible_tag_calib['rotation'][2],
                                          w=visible_tag_calib['rotation'][3])
        
        # First get base_link pose from visible tag
        T_map_visible = visible_tag_pose
        T_visible_base = self.inverse_transform(T_base_visible)
        T_map_base = self.compose_transforms(T_map_visible, T_visible_base)
        
        # Now predict other tags using base_link pose
        for tag_id, tag_calib in self.static_transforms.items():
            if tag_id == visible_tag_id:
                continue
                
            # Get tag calibration
            T_base_tag = Transform()
            T_base_tag.translation = Vector3(x=tag_calib['translation'][0],
                                          y=tag_calib['translation'][1],
                                          z=tag_calib['translation'][2])
            T_base_tag.rotation = Quaternion(x=tag_calib['rotation'][0],
                                          y=tag_calib['rotation'][1],
                                          z=tag_calib['rotation'][2],
                                          w=tag_calib['rotation'][3])
            
            # Predict tag pose: map->base->tag
            T_map_tag = self.compose_transforms(T_map_base, T_base_tag)
            predictions[tag_id] = T_map_tag
            
        return predictions

    def compute_baselink_pose(self):
        """Compute base_link pose from available tag detections"""
        transforms = []
        detected_tags = {}
        
        # First collect all actual detections
        for tag_id, detections in self.latest_detections.items():
            if not detections:
                continue

            try:
                _, transform_stamped = detections[-1]
                detected_tags[tag_id] = transform_stamped.transform
                
            except Exception as e:
                self.get_logger().error(f'Error processing tag {tag_id}: {str(e)}')
                continue
        
        # If we have detections, predict occluded tags
        predicted_tags = {}
        if detected_tags:
            # Use each visible tag to predict others and collect all predictions
            for tag_id, tag_pose in detected_tags.items():
                predictions = self.predict_occluded_tag(tag_id, tag_pose)
                for pred_tag_id, pred_pose in predictions.items():
                    if pred_tag_id not in predicted_tags:
                        predicted_tags[pred_tag_id] = []
                    predicted_tags[pred_tag_id].append(pred_pose)
        
        # Process actual detections
        for tag_id, transform in detected_tags.items():
            try:
                tag_transform = self.static_transforms[tag_id]
                T_base_tag = Transform()
                T_base_tag.translation = Vector3(x=tag_transform['translation'][0],
                                              y=tag_transform['translation'][1],
                                              z=tag_transform['translation'][2])
                T_base_tag.rotation = Quaternion(x=tag_transform['rotation'][0],
                                              y=tag_transform['rotation'][1],
                                              z=tag_transform['rotation'][2],
                                              w=tag_transform['rotation'][3])
                
                T_tag_base = self.inverse_transform(T_base_tag)
                T_map_base = self.compose_transforms(transform, T_tag_base)
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.pose.position = Point(x=T_map_base.translation.x,
                                        y=T_map_base.translation.y,
                                        z=T_map_base.translation.z)
                pose.pose.pose.orientation = T_map_base.rotation

                transforms.append((pose, 1.0))  # Weight of 1.0 for real detections
                
            except Exception as e:
                self.get_logger().error(f'Error processing tag {tag_id}: {str(e)}')
                continue
        
        # Add predicted poses with lower weights
        for tag_id, predictions in predicted_tags.items():
            if tag_id in detected_tags:  # Skip if we already have a real detection
                continue
                
            if predictions:
                # Average the predictions for this tag
                pos_sum = np.zeros(3)
                quats = []
                for pred in predictions:
                    pos_sum += np.array([pred.translation.x,
                                       pred.translation.y,
                                       pred.translation.z])
                    quats.append([pred.rotation.x,
                                pred.rotation.y,
                                pred.rotation.z,
                                pred.rotation.w])
                
                avg_pos = pos_sum / len(predictions)
                avg_rot = Rotation.from_quat(quats).mean()
                avg_quat = avg_rot.as_quat()
                
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.pose.position = Point(x=float(avg_pos[0]),
                                         y=float(avg_pos[1]),
                                         z=float(avg_pos[2]))
                pose.pose.pose.orientation = Quaternion(x=float(avg_quat[0]),
                                                 y=float(avg_quat[1]),
                                                 z=float(avg_quat[2]),
                                                 w=float(avg_quat[3]))
                
                transforms.append((pose, 0.5))  # Weight of 0.5 for predicted poses
        
        if not transforms:
            return None
        
        # Weighted average of all poses
        total_weight = sum(w for _, w in transforms)
        pos_sum = np.zeros(3)
        quats = []
        weights = []
        
        for pose, weight in transforms:
            norm_weight = weight / total_weight
            pos_sum += norm_weight * np.array([pose.pose.pose.position.x,
                                             pose.pose.pose.position.y,
                                             pose.pose.pose.position.z])
            quats.append([pose.pose.pose.orientation.x,
                         pose.pose.pose.orientation.y,
                         pose.pose.pose.orientation.z,
                         pose.pose.pose.orientation.w])
            weights.append(norm_weight)
        
        # Average rotation using scipy and apply additional -90 degree Z rotation
        avg_rot = Rotation.from_quat(quats).mean(weights=weights)
        rot_z = Rotation.from_euler('z', -np.pi/2)
        final_rot = avg_rot * rot_z
        final_quat = final_rot.as_quat()
        
        # Create final pose
        final_pose = PoseWithCovarianceStamped()
        final_pose.header.frame_id = 'map'
        final_pose.header.stamp = self.get_clock().now().to_msg()
        final_pose.pose.pose.position = Point(x=float(pos_sum[0]),
                                       y=float(pos_sum[1]),
                                       z=float(pos_sum[2]))
        final_pose.pose.pose.orientation = self.imu_orrientation
        
        final_pose.pose.covariance = [
                1e-9,0.0, 0.0, 0.0, 0.0, 0.0,  # X, Y, Z
                0.0, 1e-9, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1e-9, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1e-5, 0.0, 0.0,  # Roll, Pitch, Yaw
                0.0, 0.0, 0.0, 0.0, 1e-5, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1e-5
                ]        
        return final_pose

    def update_robot_state(self):
        """Update and publish robot state"""
        if not self.latest_detections:
            return
        
        baselink_pose = self.compute_baselink_pose()
        if baselink_pose:
            # Publish pose for EKF
            self.pose_pub.publish(baselink_pose)
            self.get_logger().debug('Published map->base_link pose')
            
            # Publish debug TF if enabled
            if self.publish_debug_tf:
                debug_tf = self.pose_to_transform_stamped(baselink_pose)
                self.tf_broadcaster.sendTransform(debug_tf)
                self.get_logger().debug('Published debug TF: map->base_link_estimated')

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