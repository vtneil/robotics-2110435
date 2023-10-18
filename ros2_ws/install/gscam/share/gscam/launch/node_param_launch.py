"""
Launch gscam_main (turn off IPC) Node(s) with parameters
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Location of configuration directory
config_dir = os.path.join(get_package_share_directory('gscam'), 'cfg')
print(config_dir)

# Parameters file
params_file = os.path.join(config_dir, 'params.yaml')
print(params_file)

# Camera calibration file
camera_config = 'file://' + os.path.join(config_dir, 'camera_parameters.ini')
print(camera_config)

# Set camera namespace
camera_name1 = 'front_camera'

def generate_launch_description():
    front_camera_gscam = Node(
        package='gscam',
        executable='gscam_node',
        output='screen',
        name='gscam_publisher',
        namespace=camera_name1,
        parameters=[
            params_file,
            {
                'camera_name': camera_name1,
                'camera_info_url': camera_config,
            },
        ],
    )

    return LaunchDescription([front_camera_gscam])
