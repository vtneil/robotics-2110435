#!/usr/bin/env python3
"""
Aruco Detection
- RTSP Stream Directly
"""
# Author : Theppasith N. <theppasith.n@obodroid.com>
# Date : 12-May-2021
##############################################################################
# Imports
##############################################################################
import math
import threading
import numpy as np
import cv2
import cv2.aruco as aruco
import os
import yaml
import aruco_controller.transformations as transform3d

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Vector3
from sensor_msgs.msg import CompressedImage, CameraInfo
from aruco_controller.detection_param import ArucoDetectionParameter
import tf2_ros
from ament_index_python.packages import get_package_share_directory


def _reorder_output_quaternion(quaternion):
    quaternion_reordered = np.array([0.0, 0.0, 0.0, 1.0])
    quaternion_reordered[0] = quaternion[1]
    quaternion_reordered[1] = quaternion[2]
    quaternion_reordered[2] = quaternion[3]
    quaternion_reordered[3] = quaternion[0]
    return quaternion_reordered
def _reorder_input_quaternion(quaternion):
    quaternion_reordered = np.array([0.0, 0.0, 0.0, 1.0])
    quaternion_reordered[0] = quaternion[3]
    quaternion_reordered[1] = quaternion[0]
    quaternion_reordered[2] = quaternion[1]
    quaternion_reordered[3] = quaternion[2]
    return quaternion_reordered
def quaternion_about_axis(angle, axis):
    return _reorder_output_quaternion(transform3d.quaternion_about_axis(angle, axis))
def quaternion_from_matrix(matrix, isprecise=False):
    return _reorder_output_quaternion(transform3d.quaternion_from_matrix(matrix, isprecise))
def identity_matrix():
    return transform3d.identity_matrix()
def quaternion_multiply(quaternion1, quaternion0):
    quaternion1 = _reorder_input_quaternion(quaternion1)
    quaternion0 = _reorder_input_quaternion(quaternion0)
    return _reorder_output_quaternion(transform3d.quaternion_multiply(quaternion1, quaternion0))
def euler_from_quaternion(qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    q = [qw, qx, qy, qz]
    return transform3d.euler_from_quaternion(q)
def pose_with_cov_from_transform_stamped(transform_stamped):
    pose_with_cov = PoseWithCovarianceStamped()
    pose_with_cov.header.frame_id = transform_stamped.header.frame_id
    pose_with_cov.pose.pose.position.x = transform_stamped.transform.translation.x
    pose_with_cov.pose.pose.position.y = transform_stamped.transform.translation.y
    pose_with_cov.pose.pose.position.z = transform_stamped.transform.translation.z
    pose_with_cov.pose.pose.orientation.x = transform_stamped.transform.rotation.x
    pose_with_cov.pose.pose.orientation.y = transform_stamped.transform.rotation.y
    pose_with_cov.pose.pose.orientation.z = transform_stamped.transform.rotation.z
    pose_with_cov.pose.pose.orientation.w = transform_stamped.transform.rotation.w
    pose_with_cov.pose.covariance[6*0+0] = 0.5 * 0.5
    pose_with_cov.pose.covariance[6*1+1] = 0.5 * 0.5
    pose_with_cov.pose.covariance[6*5+5] = math.pi/12.0 * math.pi/12.0
    return pose_with_cov
def cv_quaternion_to_ros(quaternion):
    """
    Convert Quaternion in OpenCV Camera Frame into X-Front Y-Right Frame
    """
    roll_90 = quaternion_about_axis(-(math.pi/2.0), (1, 0, 0))
    yaw_90 = quaternion_about_axis(-(math.pi/2.0), (0, 0, 1))
    spin = quaternion_multiply(roll_90, yaw_90)
    result = quaternion_multiply(quaternion, spin)
    return result

class ArucoDetector(Node):
    """
    Class to Detect Aruco From RTSP Stream
    """
    def __init__(self):
        super().__init__("aruco_detector")
        self.node_name = "ARUCO-DETECTOR"

        # Selector
        self.show_visualization = False
        self.start_active = True
        self.publish_rotated_tf = False
        self.publish_tf = True

        # Variables
        self.active = self.start_active
        self.found = False
        self.camera_ready = False

        package_share_directory = get_package_share_directory('aruco_controller')
        self.path = package_share_directory + "/cfg/"
        self.read_marker_param(self.path + "marker_param.yaml")
        self.marker_id = [int(os.getenv('ROS_DOMAIN_ID'))] #self.marker_ids
        self.log("Usng Aruco NO. [{}]".format(self.marker_id))

        # Transform Broadcaster
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer, self)

        # Aruco Stuff
        #self.aruco_dict = aruco.Dictionary_get(7)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.param_class = ArucoDetectionParameter()
        self.detection_param = self.param_class.create_cv_aruco_detector_param()

        # Camera Stuff
        self.camera_matrix = None
        self.distortion_coef = None

        # Frame
        self.raw_image_header = None
        self.frame = None
        self.rgb_frame = None
        self.lock = threading.Lock()

        # Image Subscriber
        self.create_subscription(
            CompressedImage,
            "/front_camera/camera/image_raw/compressed",
            self.image_callback,
            1
        )

        # Camera Info Subscriber
        self.create_subscription(
            CameraInfo,
            "/front_camera/camera/image_raw/camera_info",
            self.camera_info_callback,
            1
        )

        # Services
        self.marker_detect_enable_topic = "/enable_detections"
        self.create_service(SetBool, self.marker_detect_enable_topic, self.set_active_service)

        # Status Pub
        self.status_pub = self.create_publisher(Bool, "~/status", 1)
        self.num_pub = self.create_publisher(Vector3, "~/number", 1)
        self.create_timer(1, self.status_report_routine)

        # Publisher
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, "~/marker", 1)

        # Processing Loop
        hz = 20
        self.create_timer(1/hz, self.process)

        # Prompt
        self.log("OBO Aruco Detector Initialization Complete")
        self.log("Start with The ACTIVE : {}".format(self.active))

    def read_marker_param(self, yaml_path):
        with open(yaml_path, "r") as yaml_file:
            yaml_instance = yaml.safe_load(yaml_file)
        self.marker_ids = list(yaml_instance['marker_id'])
        self.marker_side_length = float(yaml_instance['marker_side_length'])

    def detect_marker(self):
        """
        Detect Marker within global self.frame
        ---
        Return: Rvec, Tvec (CV:Rodrigues)
        """
        if self.frame is None:
            self.log("Empty Frame Found - Return False")
            return False, None, None, None
        # Aruco Detection
        corners, ids, _ = aruco.detectMarkers(
            self.frame,
            self.aruco_dict,
            #parameters=self.detection_param,
            # cameraMatrix=self.camera_matrix,
            # distCoeff=self.distortion_coef
        )
        if len(corners) == 0:
            self.log("No Marker Detected ! - Return False")
            self.found = False
            return False, None, None, None

        # Check Detection Result
        detection_found = np.all(ids is not None)
        if detection_found:
            self.found = True
            self.log("All Detected Markers : {}".format(ids))
            # Query Result
            for id in self.marker_id:
                # query from sequence of marker_id
                idx_result, _ = self.filter_array(ids, id)
                found_result = idx_result.size > 0
                # Found Marker
                if found_result:
                    # Pose Estimation
                    self.log("Create Detection Result for Marker : {}  ".format(id))
                    target_index = idx_result[0] # Select First Index
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                        corners[target_index],
                        self.marker_side_length, # Side Length in metre
                        cameraMatrix=self.camera_matrix,
                        distCoeffs=self.distortion_coef
                    )
                    # Suppress Warning
                    (rvec - tvec).any()
                    # Visualization
                    if self.show_visualization:
                        self.draw_visulization(self.rgb_frame, rvec, tvec, corners)
                    # Return Result
                    return True, rvec, tvec, "fiducial"

            # Found Wrong Number
            self.log("marker_id : {} Not in target - skip".format(ids))
            return False, None, None, None

        return False, None, None, None

    def filter_array(self, numpy_array ,marker_id):
        """
        Create Criteria for Searching Marker
        ---
        Tuple ([idx], [sub-idx])
        eg. [ [1,3],[3,4] ]
        find 3 ->  ([0,1],[1,0])
        array found [0,1]
        found at idx [1,0]
        """
        return np.where(numpy_array == marker_id)

    def process(self):
        """
        Process The Aruco Detection within the Specified Rate
        """
        if self.active is False:
            if self.frame is not None:
                self.frame = None
            return
        if self.frame is None:
            self.log("Empty Frame Found - Processing Loop Skip")
            return

        # Aruco Detection
        marker_found, rvec, tvec, frame_name = self.detect_marker()
        if marker_found:
            # Put Rvec Tvec into the KALMAN FILTER
            # self.update_kalman(rvec, tvec)
            # Publish Detection Result
            self.pub_detections(rvec, tvec, frame_name)
        else:
            self.get_logger().error("Marker Not Found")

        # Visualization
        if self.show_visualization and self.active:
            cv2.imshow('rgb_frame', self.rgb_frame)
            cv2.waitKey(1)

    def set_active_service(self, request, response):
        """
        Service to enable / disable this module
        """
        self.log("\n\n\n\n")
        requester = self.get_caller_id()
        self.log("Incoming Request from {} to Set Active to : {}".format(requester, request.data))
        self.active = bool(request.data)
        self.log("Set Active to : {}".format(self.active))

        # response = SetBoolResponse()
        response.success = True
        response.message = "Set active status to : {}".format(self.active)
        return response

    def get_caller_id(self):
        """
        Return : List of thing related to enable/disable aruco sevice
        """
        node_names = self.get_node_names()
        for name in node_names:
            if name != self.get_name():
                names =[]
                names.append(str(name))
                return str(names)
        return ''

    def status_report_routine(self):
        result = Bool()
        result.data = self.active
        self.status_pub.publish(result)

    def pub_detections(self, rvec, tvec, frame_name):
        """
        Publish Detections
        """
        # Create TF for detections
        marker_tf, rotated_marker_tf = self.create_marker_tf(rvec[0][0], tvec[0][0], frame_name) # marker_tf (Z-Front), rotated_marker_tf (X-Front)
        # Publish Pose
        self.pub_pose_cov(rotated_marker_tf)
        # Publish TF
        if self.publish_tf:
            self.tf2_broadcaster.sendTransform(marker_tf)
            self.tf2_broadcaster.sendTransform(rotated_marker_tf)

    def pub_pose_cov(self, marker_tf):
        """
        Publish Pose with CovarianceStamped of marker
        """
        pose_cov = pose_with_cov_from_transform_stamped(marker_tf)
        pose_cov.header.stamp = marker_tf.header.stamp # timeStamp
        # Covariance of Detection
        pose_cov.pose.covariance[6*0+0] = 0.1 * 0.1
        pose_cov.pose.covariance[6*1+1] = 0.1 * 0.1
        pose_cov.pose.covariance[6*5+5] = math.pi/24.0 * math.pi/24.0
        # Publish Pose
        self.pose_publisher.publish(pose_cov)
        # TODO : DEBUG
        _, _, yaw = euler_from_quaternion(
            pose_cov.pose.pose.orientation.x,
            pose_cov.pose.pose.orientation.y,
            pose_cov.pose.pose.orientation.z,
            pose_cov.pose.pose.orientation.w
        )
        num = Vector3()
        num.x = pose_cov.pose.pose.position.x
        num.y = pose_cov.pose.pose.position.y
        num.z = yaw
        self.num_pub.publish(num)

    def create_marker_tf(self, rvec, tvec, frame_id):
        """
        Create Station TF
        """
        trans = TransformStamped()
        trans.header.stamp = self.get_clock().now().to_msg()
        trans.header.frame_id = self.camera_frame_id # Parent
        trans.child_frame_id = frame_id # Child is "fiducial_"+str(id) (Z-Front)

        # Translation Vector X-Right Y-Up Z-pointing out
        trans.transform.translation.x = tvec[0]
        trans.transform.translation.y = tvec[1]
        trans.transform.translation.z = tvec[2]

        # Rotation Vector to 4x4 Homogeneous Transformation (Rotation Only)
        rotation_matrix = identity_matrix()
        rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)
        quaternion = quaternion_from_matrix(rotation_matrix)

        # Old Pitch
        old_pitch = euler_from_quaternion(
            quaternion[0],
            quaternion[1],
            quaternion[2],
            quaternion[3],
        )[1]

        trans.transform.rotation.x = quaternion[0]
        trans.transform.rotation.y = quaternion[1]
        trans.transform.rotation.z = quaternion[2]
        trans.transform.rotation.w = quaternion[3]

        # Rotated to ROS X-Front Y-Left Z-Up Version
        # trans_rotated = trans
        trans_rotated = TransformStamped()
        trans_rotated.transform.translation.x = tvec[0]
        trans_rotated.transform.translation.y = tvec[1]
        trans_rotated.transform.translation.z = tvec[2]

        # Add Little Epsilon deltaTime for prevent TF_REPEATED Publishing
        trans_rotated.header.stamp = (self.get_clock().now() + rclpy.duration.Duration(seconds=0.001)).to_msg()
        # Rotated Frame Id
        trans_rotated.header.frame_id = self.camera_frame_id
        trans_rotated.child_frame_id = frame_id + "_rotated"
        quaternion = cv_quaternion_to_ros(quaternion)
        # Suppress Pitch and Roll to have YAW Only
        quaternion_yaw = quaternion_from_euler(0.0, old_pitch-math.pi/2.0, 0.0)

        trans_rotated.transform.rotation.x = quaternion_yaw[0]
        trans_rotated.transform.rotation.y = quaternion_yaw[1]
        trans_rotated.transform.rotation.z = quaternion_yaw[2]
        trans_rotated.transform.rotation.w = quaternion_yaw[3]

        return trans, trans_rotated

    def camera_info_callback(self, msg):
        """
        on Received CameraInfo
        """
        if not self.active:
            return
        if self.camera_ready:
            # Skip Assignment if we've already got data
            return

        self.log("Received Following Camera Info : {}".format(msg.header.frame_id))

        self.camera_matrix = np.array(msg.k).reshape(3,3)
        self.distortion_coef = np.array(msg.d)

        self.log("Camera Matrix -> : \n {}".format(self.camera_matrix))
        self.log("Distortion Matrix -> : \n {}".format(self.distortion_coef))

        self.camera_ready = True

    def image_callback(self, msg):
        """
        on Received Image
        """
        if not self.active:
            return

        # Check if we got the camera matrix
        if not self.camera_ready:
            self.log("Received Image when Camera Information is not Ready - Skip Frame")
            # Try to Receive the camera info here
            return

        # Create NP Array to Hold Jpeg Pic
        raw_arr = np.frombuffer(msg.data, dtype=np.uint8)
        self.camera_frame_id = msg.header.frame_id
        # Decode Jpeg Pic to OpenCV Format
        self.lock.acquire()
        self.raw_image_header = msg.header
        self.frame = cv2.imdecode(raw_arr, cv2.IMREAD_GRAYSCALE)
        # image decode to rgb_frame when would like show_visualization
        if self.show_visualization:
            self.rgb_frame = cv2.imdecode(raw_arr, cv2.IMREAD_COLOR)
        self.lock.release()

    def draw_visulization(self, frame, rvec, tvec, corners):
        """
        Draw Visualization On Image Frame
        """
        aruco.drawDetectedMarkers(
            frame,
            corners
        )  # Draw A square around the markers

        # aruco.drawAxis(
        #     frame,
        #     self.camera_matrix,
        #     self.distortion_coef,
        #     rvec,
        #     tvec,
        #     0.07
        # )
        cv2.drawFrameAxes(frame, self.camera_matrix, self.distortion_coef, rvec, tvec, length=0.07, thickness=3)

    def log(self, msg):
        self.get_logger().info("[{}] : {}".format(self.node_name, msg))

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    # Remove OpenCV Windows when visualization
    if node.show_visualization:
        cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
