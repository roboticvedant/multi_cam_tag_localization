# ------------------------------------------------------------------------------
# This file is part of **extrinsic-calibrator:
# October 2024
# Copyright 2024 IKERLAN. All Rights Reserved.
#
#
# LICENSE NOTICE
#
# This software is available under a dual license system. Choose between:
# - GNU Affero General Public License v3.0 for open-source usage, or
# - A commercial license for proprietary development.
# For commercial license details, contact us at info@ikerlan.es.
#
# GNU Affero General Public License v3.0
# Version 3, 19 November 2007
# © 2007 Free Software Foundation, Inc. <https://fsf.org/>
#
# Licensed under a dual license system:
# 1. Open-source usage under the GNU Affero General Public License v3.0
#    (AGPL-3.0), allowing you to freely use, modify, and distribute the
#    software for open-source projects. You can find a copy of the AGPL-3.0
#    license at https://www.gnu.org/licenses/agpl-3.0.html.
# 2. For commercial/proprietary use, a separate commercial license is required.
#    Please contact us at info@ikerlan.es for inquiries about our commercial
#    licensing options.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# Author Information:
# Author: Josep Rueda Collell
# Created: October 2024
# Affiliation: IKERLAN (https://www.ikerlan.es)
# ------------------------------------------------------------------------------

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'apriltag_localize'
    config_dir = os.path.join(get_package_share_directory(package_name), 'config', 'extrinsic_calib')

    webCam_config = os.path.join(config_dir, 'webCam.yaml')
    lenovoCam_config = os.path.join(config_dir, 'lenovoCam.yaml')

    return LaunchDescription([
        # Node(
        #     package='usb_cam',
        #     executable='usb_cam_node_exe',
        #     name='web_camera',
        #     namespace='camera_1',
        #     parameters=[webCam_config],
        #     output='screen'
        # ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='lenovo_camera',
            namespace='camera_1',
            parameters=[lenovoCam_config],
            output='screen'
        )
    ])
