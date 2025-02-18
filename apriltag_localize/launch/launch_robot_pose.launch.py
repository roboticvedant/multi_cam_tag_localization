from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory("apriltag_localize")
    
    # Load YAML configurations
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        # Start robot state estimator first
        Node(
            package='apriltag_localize',
            executable='robot_state_estimator',
            name='robot_state_estimator',
            output='screen'
        ),

        Node(
            package='apriltag_localize',
            executable='kalman_filter',
            name='filter_node',
            output='screen'
        ),
        
        # # Local EKF - provides odom->base_link transform
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node_odom',
        #     output='screen',
        #     parameters=[ekf_config],
        #     remappings=[('/odometry/filtered', '/odometry/local')]
        # ),
        
        # Global EKF (map → odom)
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='global_ekf',
        #     output='screen',
        #     parameters=[ekf_config],
        #     remappings=[
        #         ('odometry/filtered', '/global_odom'),
        #         ('/set_pose', '/global_ekf/set_pose')
        #     ]
        # ),
        
        # Local EKF (odom → base_link)
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='local_ekf',
        #     output='screen',
        #     parameters=[ekf_config],
        #     remappings=[
        #         ('odometry/filtered', '/local_odom'),
        #         ('/set_pose', '/local_ekf/set_pose'),
        #         ('/diagnostics', '/local_ekf/diagnostics')
        #     ],
        # ),
        
        # # Initial Pose Setter
        # Node(
        #     package='apriltag_localize',
        #     executable='initial_pose_setter',
        #     name='initial_pose_setter',
        #     output='screen'
        # ),
    ])