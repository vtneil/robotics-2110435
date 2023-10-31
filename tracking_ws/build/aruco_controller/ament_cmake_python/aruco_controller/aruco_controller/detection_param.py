#!/usr/bin/env python3
"""
Aruco Detection
Parameter Reader File
- Create Opencv's Aruco Detector Parameter instance
- Read param from file
"""
# Author : Theppasith N. <theppasith.n@obodroid.com>
# Date : 13-May-2021
##############################################################################
# Imports
##############################################################################
# import cv2.aruco as aruco
# import rospy
# import yaml
# import rospkg
import cv2.aruco as aruco
# import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory

class ArucoDetectionParameter(object):
    """
    Class to Manage Aruco Detection Parameter
    """
    def __init__(self):
        # Parameter Dict
        self.param_dict = self.create_default_param_dict()

        # Path Solver
        package_share_directory = get_package_share_directory('aruco_controller')
        self.path = package_share_directory + "/cfg/" # End with '/'

        # Initialize DetectionParam
        self.detection_param = aruco.DetectorParameters_create()
        self.assign_to_cv_aruco()

        # Fill Content by File
        #self.read_from_file(self.path + "aruco_detect_param.yaml")
        self.read_from_file(self.path + "default_param.yaml")

    def create_default_param_dict(self):
        """
        Fill The Default Value to the Dict
        """
        param_dict = dict()
        # Fill the Dict with Default Parameters
        param_dict["adaptiveThreshConstant"] = 7
        param_dict["adaptiveThreshWinSizeMin"] = 3
        param_dict["adaptiveThreshWinSizeMax"] = 23
        param_dict["adaptiveThreshWinSizeStep"] = 10
        param_dict["cornerRefinementMaxIterations"] = 30
        param_dict["cornerRefinementMinAccuracy"] = 0.1
        param_dict["cornerRefinementWinSize"] = 5
        param_dict["cornerRefinementMethod"] = 0
        param_dict["errorCorrectionRate"] = 0.6
        param_dict["minCornerDistanceRate"] = 0.05
        param_dict["markerBorderBits"] = 1
        param_dict["maxErroneousBitsInBorderRate"] = 0.35
        param_dict["minDistanceToBorder"] = 3
        param_dict["minMarkerDistanceRate"] = 0.05
        param_dict["minMarkerPerimeterRate"] = 0.03
        param_dict["maxMarkerPerimeterRate"] = 4.0
        param_dict["minOtsuStdDev"] = 5.0
        param_dict["perspectiveRemoveIgnoredMarginPerCell"] = 0.13
        param_dict["perspectiveRemovePixelPerCell"] = 4
        param_dict["polygonalApproxAccuracyRate"] = 0.03

        return param_dict

    def assign_to_cv_aruco(self):
        """
        Assign param_dict to Aruco Detection Param
        """
        # Iterate through item in dict
        for key, val in list(self.param_dict.items()):
            # Create Attribute with key as name and value assigned
            setattr(self.detection_param, key, val)

    def create_cv_aruco_detector_param(self):
        """
        Create OpenCV's Aruco Detector Param Instance
        - Assign data from self.param_dict to OpenCV Aruco
        - Return "OpenCV Arudo Param"
        """
        # Create Detection from Dict
        self.assign_to_cv_aruco()
        for key, val in list(self.param_dict.items()):
            print("{} : {}".format(key, val))
        return self.detection_param

    def read_from_file(self, yaml_path):
        """
        Read YAML From file and Create The Dict
        """
        # Reset the Param Dict
        self.param_dict = self.create_default_param_dict()
        # Open YAML File and Create Dict
        with open(yaml_path, "r") as yaml_file:
            yaml_instance = yaml.safe_load(yaml_file)
            # Iterate and copy field that is exist in default dict
            for key, val in list(yaml_instance.items()):
                if key in self.param_dict:
                    self.param_dict[key] = val
