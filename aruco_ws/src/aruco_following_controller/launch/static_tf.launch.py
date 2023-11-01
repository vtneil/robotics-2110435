from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    base_link_front_camera = Node(package = "tf2_ros", 
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0.1", "0", "0", "0", "base_link", "front_camera"])
    front_camera_front_camera_optical = Node(package = "tf2_ros", 
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "0.5", "-0.5", "0.5", "-0.5", "front_camera", "front_camera_optical"])

    ld.add_action(base_link_front_camera)
    ld.add_action(front_camera_front_camera_optical)

    return ld