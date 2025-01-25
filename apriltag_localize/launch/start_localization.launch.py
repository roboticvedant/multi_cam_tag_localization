
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

    
    return LaunchDescription([

        # source the ros2 ws that contains the extrinsic_calibrator_core package

        # Laucnh the set of usb-cameras with their own config_files
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("apriltag_localize"),
                "launch",
                "launch_usb_cameras.launch.py"]))
        ),
        
        # Laucnh the rviz visualizer with the TF of the map and the cameras
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("apriltag_localize"),
                "launch",
                "launch_rviz.launch.py"]))
        ),
        
        # Launch the extrinsic calibrator node. The config file is in the config folder and is passed to the node using the generate_parameter_library
        Node(
            package='extrinsic_calibrator_core',
            executable='extrinsic_calibrator_node.py',
            name='extrinsic_calibrator_node',
            output='screen',
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
            executable='multi_detect_agg',
            name='agg_node',
            output='screen',
            parameters=[{'cameras': ['camera_1', 'camera_2']}]
        ), 
        
        
       
    ])

