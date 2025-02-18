import numpy as np
from collections import deque
from scipy.spatial.transform import Rotation

class LowPassFilter:
    def __init__(self, alpha, median_window):
        self.alpha = alpha
        self.previous = None
        self.median_window = median_window
        self.median_buffer = deque(maxlen=median_window)

    def apply(self, current):
        self.median_buffer.append(current)

        if len(self.median_buffer) == self.median_window:
            median_filtered = np.median(self.median_buffer)
        else:
            median_filtered = current

        if self.previous is None:
            self.previous = median_filtered
            
        filtered = self.alpha * median_filtered + (1 - self.alpha) * self.previous
        self.previous = filtered

        return filtered

    def reset(self):
        self.previous = None
        self.median_buffer.clear()

class VectorLowPassFilter:
    def __init__(self, alpha=0.1, median_window=1):
        self.filter_x = LowPassFilter(alpha, median_window)
        self.filter_y = LowPassFilter(alpha, median_window)
        self.filter_z = LowPassFilter(alpha, median_window)

    def filter(self, vector_data):
        if isinstance(vector_data, dict):
            filtered_data = {
                'x': self.filter_x.apply(vector_data['x']),
                'y': self.filter_y.apply(vector_data['y']),
                'z': self.filter_z.apply(vector_data['z']),
            }
        elif isinstance(vector_data, (list, np.ndarray)):
            filtered_data = {
                'x': self.filter_x.apply(vector_data[0]),
                'y': self.filter_y.apply(vector_data[1]),
                'z': self.filter_z.apply(vector_data[2]),
            }
        else:
            raise TypeError("vector_data must be a dictionary with 'x', 'y', 'z' keys or a sequence.")
        
        return filtered_data

    def reset(self):
        self.filter_x.reset()
        self.filter_y.reset()
        self.filter_z.reset()

class PoseFilter:
    def __init__(self, position_alpha=0.1, orientation_alpha=0.1, 
                 position_median_window=1, orientation_median_window=1):
        """
        Initialize filters for pose data (position and orientation).
        
        Args:
            position_alpha (float): Smoothing factor for position (0 < alpha <= 1)
            orientation_alpha (float): Smoothing factor for orientation (0 < alpha <= 1)
            position_median_window (int): Window size for position median filter
            orientation_median_window (int): Window size for orientation median filter
        """
        self.position_filter = VectorLowPassFilter(position_alpha, position_median_window)
        self.orientation_filter = VectorLowPassFilter(orientation_alpha, orientation_median_window)
        
    def filter(self, pose_data):
        """
        Filter pose data including position and orientation.
        
        Args:
            pose_data: Dictionary containing:
                - 'position': dict with 'x', 'y', 'z' or array [x, y, z]
                - 'orientation': dict with 'x', 'y', 'z', 'w' or array [x, y, z, w] (quaternion)
        
        Returns:
            dict: Filtered pose data with position and orientation
        """
        # Filter position
        if 'position' in pose_data:
            filtered_position = self.position_filter.filter(pose_data['position'])
        else:
            raise KeyError("pose_data must contain 'position'")

        # Handle orientation filtering in euler space
        if 'orientation' in pose_data:
            quat = pose_data['orientation']
            if isinstance(quat, dict):
                quat = [quat['x'], quat['y'], quat['z'], quat['w']]
            
            # Convert quaternion to euler angles
            euler = Rotation.from_quat(quat).as_euler('xyz')
            
            # Filter euler angles
            filtered_euler = self.orientation_filter.filter(euler)
            
            # Convert back to quaternion
            filtered_quat = Rotation.from_euler('xyz', 
                [filtered_euler['x'], filtered_euler['y'], filtered_euler['z']]).as_quat()
            
            filtered_orientation = {
                'x': filtered_quat[0],
                'y': filtered_quat[1],
                'z': filtered_quat[2],
                'w': filtered_quat[3]
            }
        else:
            raise KeyError("pose_data must contain 'orientation'")

        return {
            'position': filtered_position,
            'orientation': filtered_orientation
        }

    def reset(self):
        """Reset all filters"""
        self.position_filter.reset()
        self.orientation_filter.reset()


class TagPoseFilter:
    def __init__(self, alpha_pos=0.1, alpha_orient=0.1, 
                 pos_median_window=1, orient_median_window=1):
        """
        Initialize a filter manager for multiple tag poses.
        
        Args:
            alpha_pos (float): Position smoothing factor
            alpha_orient (float): Orientation smoothing factor
            pos_median_window (int): Position median filter window
            orient_median_window (int): Orientation median filter window
        """
        self.filters = {}
        self.alpha_pos = alpha_pos
        self.alpha_orient = alpha_orient
        self.pos_median_window = pos_median_window
        self.orient_median_window = orient_median_window

    def get_filter(self, tag_id):
        """Get or create filter for a specific tag"""
        if tag_id not in self.filters:
            self.filters[tag_id] = PoseFilter(
                position_alpha=self.alpha_pos,
                orientation_alpha=self.alpha_orient,
                position_median_window=self.pos_median_window,
                orientation_median_window=self.orient_median_window
            )
        return self.filters[tag_id]

    def filter_pose(self, tag_id, pose_data):
        """
        Filter pose data for a specific tag.
        
        Args:
            tag_id: Tag identifier
            pose_data: Pose data dictionary with position and orientation
        
        Returns:
            dict: Filtered pose data
        """
        tag_filter = self.get_filter(tag_id)
        return tag_filter.filter(pose_data)

    def reset_filter(self, tag_id):
        """Reset filter for a specific tag"""
        if tag_id in self.filters:
            self.filters[tag_id].reset()

    def reset_all(self):
        """Reset all tag filters"""
        for filter in self.filters.values():
            filter.reset()