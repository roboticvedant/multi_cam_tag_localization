
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Get the package share directory for config files
    apriltag_localize_share = get_package_share_directory('apriltag_localize')

    # Define paths to the configuration files
    tag_detect_common_config = os.path.join(apriltag_localize_share, 'config','tag_detect', 'common_config.yaml')
    tag_detect_camera1_config = os.path.join(apriltag_localize_share, 'config','tag_detect', 'camera1_config.yaml')
    tag_detect_camera2_config = os.path.join(apriltag_localize_share, 'config','tag_detect', 'camera2_config.yaml')
    tag_detect_camera3_config = os.path.join(apriltag_localize_share, 'config','tag_detect', 'camera3_config.yaml')


    
    return LaunchDescription([

        # Laucnh the rviz visualizer with the TF of the map and the cameras
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("apriltag_localize"),
                "launch",
                "launch_rviz.launch.py"]))
        ),
        # My custom nodes from here on:
        Node(
            package='apriltag_localize',
            executable='tag_detect',
            name='detect_node_cam1',
            output='screen',
            parameters=[tag_detect_common_config, tag_detect_camera1_config]
        ), 
        Node(
            package='apriltag_localize',
            executable='tag_detect',
            name='detect_node_cam2',
            output='screen',
            parameters=[tag_detect_common_config, tag_detect_camera2_config]
        ), 
        Node(
            package='apriltag_localize',
            executable='tag_detect',
            name='detect_node_cam3',
            output='screen',
            parameters=[tag_detect_common_config, tag_detect_camera3_config]
        ), 

         Node(
            package='apriltag_localize',
            executable='multi_detect_agg',
            name='agg_node',
            output='screen',
            parameters=[{'cameras': ['camera_1', 'camera_2', 'camera_3']}]
        ), 
        
        
       
    ])

