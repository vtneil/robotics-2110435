from typing import List, Optional
import numpy as np

from geometry_msgs.msg import Twist, Pose

import aruco_controller.transformations as transform3d

def euler_from_quaternion(qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    q = [qw, qx, qy, qz]
    return transform3d.euler_from_quaternion(q)


class ControlLaw:
    def __init__(self):
        self.kp_linear = 1.0
        self.kp_angular = 2.0
        self.goal_tolerance = 0.5

    def pose_diff(self, current_pose:Pose, goal_pose:Pose) -> np.array:
        dx = goal_pose.position.x - current_pose.position.x
        dy = goal_pose.position.y - current_pose.position.y

        return np.array([dx, dy])

    def theta_goal(self, current_pose:Pose, goal_pose:Pose):
        diff = self.pose_diff(current_pose, goal_pose)

        return np.arctan2(diff[1],diff[0])

    def go_to_pose(self, current_pose:Pose, goal_pose:Pose) -> Twist:
        diff = self.pose_diff(current_pose, goal_pose)

        err_dist = np.linalg.norm(diff)
        _,_,current_yaw = euler_from_quaternion(
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w)
        err_theta = self.theta_goal(current_pose, goal_pose) - current_yaw
        cmd_vel = Twist()
        cmd_vel.linear.x = self.kp_linear*err_dist
        cmd_vel.angular.z = self.kp_angular*np.arctan2(np.sin(err_theta),np.cos(err_theta))

        return cmd_vel

    def goal_checker(self, current_pose:Pose, goal_pose:Pose) -> bool:
        diff = self.pose_diff(current_pose, goal_pose)

        return np.linalg.norm(diff) <= self.goal_tolerance