from setuptools import find_packages, setup

package_name = 'apriltag_localize'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Config files
        ('share/' + package_name + '/config', [
            'config/ekf.yaml',
        ]),
        ('share/' + package_name + '/config/extrinsic_calib/intrinsics', [
            'config/extrinsic_calib/intrinsics/lenovoCam_intrinsics.yaml',
            'config/extrinsic_calib/intrinsics/webCam_intrinsics.yaml',
            'config/extrinsic_calib/intrinsics/nexigoCam_intrinsics.yaml'
        ]),
        ('share/' + package_name + '/config/tag_detect', [
            'config/tag_detect/common_config.yaml',
            'config/tag_detect/camera1_config.yaml',
            'config/tag_detect/camera2_config.yaml'
        ]),
        ('share/' + package_name + '/config/extrinsic_calib', [
            'config/extrinsic_calib/webCam.yaml',
            'config/extrinsic_calib/lenovoCam.yaml',
            'config/extrinsic_calib/nexigoCam.yaml'
        ]),
        # RViz configuration
        ('share/' + package_name + '/rviz', ['rviz/extrinsic.rviz']),
        # Launch files
        ('share/' + package_name + '/launch', [
            'launch/launch_extrinsic_calib.launch.py',
            'launch/launch_rviz.launch.py',
            'launch/launch_usb_cameras.launch.py',
            'launch/launch_apriltag_localize.launch.py',
            'launch/launch_robot_pose.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vnaik792014',
    maintainer_email='vnaik792014@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_detect = apriltag_localize.tag_detect:main',
            'multi_detect_agg = apriltag_localize.multi_detect_agg:main',
            'calib_check = apriltag_localize.calib_check:main',
            'image_undistort = apriltag_localize.image_undistort:main',
            'robot_calibrate = apriltag_localize.robot_calibrate:main',
            'robot_state_estimator = apriltag_localize.robot_state_estimator:main',
            'initial_pose_setter = apriltag_localize.initial_pose_setter:main',
            'map_to_odom_publisher = apriltag_localize.map_to_odom_publisher:main',
        ],
    },
)
