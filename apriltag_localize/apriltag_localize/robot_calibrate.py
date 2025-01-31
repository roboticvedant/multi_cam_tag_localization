#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import numpy as np
import yaml
from collections import defaultdict
import time
class SimpleCalibration(Node):
    def __init__(self):
        super().__init__('simple_calibration')
        self.tag_ids = ["tag_21", "tag_27", "tag_28"]
        self.samples = []
        self.current_transforms = defaultdict(list)
        self.max_time_diff = 0.01
        self.num_samples = 500
        
        self.tf_sub = self.create_subscription(
            TFMessage, 
            '/tf',
            self.tf_callback,
            10
        )

    def tf_callback(self, msg):
        if len(self.samples) >= self.num_samples:
            self.compute_final_transform()
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        for transform in msg.transforms:
            if transform.child_frame_id in self.tag_ids:
                self.current_transforms[transform.child_frame_id].append(
                    (current_time, transform)
                )
        
        for tag in self.tag_ids:
            self.current_transforms[tag] = [
                (t, transform) for t, transform in self.current_transforms[tag]
                if current_time - t < 0.1
            ]

        self._check_synchronized_transforms()

    def _check_synchronized_transforms(self):
        if any(len(transforms) == 0 for transforms in self.current_transforms.values()):
            return

        ref_tag = self.tag_ids[0]
        for ref_time, ref_transform in self.current_transforms[ref_tag]:
            synchronized_transforms = {ref_tag: ref_transform}

            for tag in self.tag_ids[1:]:
                matching_transform = None
                for t, transform in self.current_transforms[tag]:
                    if abs(t - ref_time) < self.max_time_diff:
                        matching_transform = transform
                        break
                if matching_transform:
                    synchronized_transforms[tag] = matching_transform
                else:
                    break

            if len(synchronized_transforms) == len(self.tag_ids):
                self.samples.append(synchronized_transforms)
                self.get_logger().info(f'Sample {len(self.samples)}/{self.num_samples}')
                # time.sleep(5)  # Give some time for the next transform to come in
                break

    def compute_final_transform(self):
        calibration = {'transforms': {}}
        
        # Compute mean for each tag
        tag_means = {}
        for tag in self.tag_ids:
            positions = []
            rotations = []
            for sample in self.samples:
                trans = sample[tag].transform
                positions.append([trans.translation.x, trans.translation.y, trans.translation.z])
                rotations.append([trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w])
            
            mean_pos = np.mean(positions, axis=0)
            mean_rot = np.mean(rotations, axis=0)
            mean_rot /= np.linalg.norm(mean_rot)
            tag_means[tag] = {'position': mean_pos, 'rotation': mean_rot}

        # Compute centroid
        centroid = np.mean([tag_means[tag]['position'] for tag in self.tag_ids], axis=0)
        base_pos = centroid.copy()
        base_pos[2] -= 0.65  # Base is 65cm below centroid

        # Compute transforms relative to base at origin
        for tag in self.tag_ids:
            # Position relative to base
            rel_pos = tag_means[tag]['position'] - base_pos
            
            # Rotation relative to base
            rel_rot = tag_means[tag]['rotation']
            rel_rot = np.array(rel_rot)
            
            calibration['transforms'][tag] = {
                'translation': rel_pos.tolist(),
                'rotation': rel_rot.tolist()
            }
            
            self.get_logger().info(
                f"{tag} transform: pos={rel_pos.tolist()}, rot={rel_rot.tolist()}"
            )

        with open('tag_calibration.yaml', 'w') as f:
            yaml.dump(calibration, f)
        
        rclpy.shutdown()

def main():
    rclpy.init()
    node = SimpleCalibration()
    rclpy.spin(node)

if __name__ == '__main__':
    main()