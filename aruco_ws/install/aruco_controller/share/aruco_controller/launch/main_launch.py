import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    gscam_dir = get_package_share_directory('gscam')
    gscam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gscam_dir, 'launch/node_param_launch.py')
            )
    )

    aruco_detector = Node(
        package='aruco_controller',
        executable='aruco_detector.py',
        output='screen',
        name='aruco_detector'
    )

    ld = LaunchDescription()
    ld.add_action(gscam_launch)
    ld.add_action(aruco_detector)
    return ld
