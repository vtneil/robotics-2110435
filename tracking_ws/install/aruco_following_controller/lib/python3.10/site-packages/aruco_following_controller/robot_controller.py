from typing import Iterator, List, Optional
from enum import Enum
import numpy as np
from threading import Lock

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
from rclpy.time import Duration, Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist, TransformStamped, Pose, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

from rcl_interfaces.msg import SetParametersResult, Parameter

from aruco_following_controller.control_law import ControlLaw

from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

from rcl_interfaces.msg import ParameterDescriptor


TO_SEC = 1e-9


class ControlState(Enum):
    IDLE = 0
    SCAN_ARUCO = 1
    FOLLOW_ARUCO = 2

class RobotController(Node, ControlLaw):
    def __init__(self):
        super().__init__('controller')
        self._init_ros_params()

        self.state = ControlState.SCAN_ARUCO
        self.current_pose:Optional[Pose] = None
        self.aruco_pose:Optional[PoseStamped] = None
        self.control_loop_lock = Lock()
        self.subscriber_cb_group = ReentrantCallbackGroup()
        self.service_server_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()
        self.lastest_aruco_stamped = self.get_clock().now().nanoseconds*TO_SEC

        self._init_communication()

    @property
    def is_found_aruco(self):
        return self.aruco_pose != None

    @property
    def is_aruco_timeout(self):
        current_time = self.get_clock().now().nanoseconds*TO_SEC
        time_diff = current_time - self.lastest_aruco_stamped
        # Timeout limit exceeded
        self.get_logger().info(f'{time_diff}')
        if time_diff >= self.aruco_timeout:
            self.aruco_pose = None
            return True

        return False

    def _init_ros_params(self):
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('max_linear_vel', 2.0)
        self.declare_parameter('min_linear_vel', 0.0)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('min_angular_vel', 0.0)
        self.declare_parameter('kp_linear', 1.0)
        self.declare_parameter('kp_angular', 1.0)
        self.declare_parameter('aruco_frame_id', 'front_camera')
        self.declare_parameter('robot_base_frame_id', 'base_footprint')
        self.declare_parameter('max_kp_linear', 3.0)
        self.declare_parameter('max_kp_angular', 4.0)
        self.declare_parameter('scan_around_vel', 1.0)
        self.declare_parameter('controller_frequency', 10.0)
        self.declare_parameter('aruco_pose_topic', '/aruco_detector/marker')
        self.declare_parameter('aruco_pose_timeout', 0.5)
        self.declare_parameter('aruco_pose_offset', 0.5)
        self.declare_parameter('set_active_topic', 'set_active')
        self.declare_parameter('goal_pose_tolerance', 0.5)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.min_linear_vel = self.get_parameter('min_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.min_angular_vel = self.get_parameter('min_angular_vel').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_kp_lin = self.get_parameter('max_kp_linear').value
        self.max_kp_ang = self.get_parameter('max_kp_angular').value
        self.aruco_frame = self.get_parameter('aruco_frame_id').value
        self.robot_base_frame = self.get_parameter('robot_base_frame_id').value
        self.scan_around_vel = self.get_parameter('scan_around_vel').value
        self.controller_freq = self.get_parameter('controller_frequency').value
        self.aruco_pose_topic = self.get_parameter('aruco_pose_topic').value
        self.aruco_timeout = self.get_parameter('aruco_pose_timeout').value
        self.goal_pose_offset = self.get_parameter('aruco_pose_offset').value
        self.set_active_topic = self.get_parameter('set_active_topic').value
        self.goal_tolerance = self.get_parameter('goal_pose_tolerance').value

    def _init_communication(self):
        self.tf_pub = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            liveliness=LivelinessPolicy.AUTOMATIC,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        self.odom_sub = self.create_subscription(
                            Odometry,
                            self.odom_topic,
                            self.odometry_cb,
                            qos_profile=qos_profile,
                            callback_group=self.subscriber_cb_group,
                        )

        self.aruco_pose_sub = self.create_subscription(
                                PoseWithCovarianceStamped,
                                self.aruco_pose_topic,
                                self.aruco_pose_cb,
                                qos_profile=1,
                                callback_group=self.subscriber_cb_group
                            )

        self._timer = self.create_timer(
                        1/self.controller_freq,
                        self._control_loop_timer,
                        self.timer_cb_group
                    )

        self.cmd_vel_pub = self.create_publisher(
                                Twist,
                                self.cmd_vel_topic,
                                qos_profile=1
                            )

        self.set_active_server = self.create_service(SetBool, self.set_active_topic, self.set_active_handle)

        self.goal_vis_pub = self.create_publisher(PoseStamped, '/goal_pose', qos_profile=1)

        self.add_on_set_parameters_callback(self._set_parameters_cb)

    def _set_parameters_cb(self, params:List[Parameter]):
        self.get_logger().info('parameters callback')
        for param in params:
            if param.name == 'kp_linear':
                if param.value >= 0:
                    if param.value > self.max_kp_lin:
                        self.get_logger().warn(f'{param.value} greater than max_kp_lin -> set to max_kp_lin')
                    self.kp_linear = (min(param.value, self.max_kp_lin))
                    self.get_logger().info('set kp_linear success')
                else:
                    self.get_logger().warn('kp_linear value is negative -> skipping')
            ####
            elif param.name == 'kp_angular':
                if param.value >= 0:
                    if param.value > self.max_kp_ang:
                        self.get_logger().warn(f'{param.value} greater than max_kp_ang -> set to max_kp_ang')
                    self.kp_angular = (min(param.value, self.max_kp_ang))
                    self.get_logger().info('set kp_angular success')
                else:
                    self.get_logger().warn('kp_angular value is negative -> skipping')
            ####
            elif param.name == 'max_linear_vel':
                if param.value >= 0:
                    self.max_linear_vel = param.value
                    self.get_logger().info('set max_linear_vel success')
                else:
                    self.get_logger().warn('max_linear_vel value is negative -> skipping')
            ####
            elif param.name == 'max_angular_vel':
                if param.value >= 0:
                    self.max_angular_vel = param.value
                    self.get_logger().info('set max_angular_vel success')
                else:
                    self.get_logger().warn('max_angular_vel  value is negative -> skipping')   
            ########
            elif param.name == 'scan_around_vel':
                if param.value >= 0:
                    self.scan_around_vel = param.value
                    self.get_logger().info('set scan_around_vel success')
                else:
                    self.get_logger().warn('scan_around_vel value is negative -> skipping')         
            ####
            elif param.name == 'aruco_pose_timeout':
                if param.value >= 0:
                    self.aruco_timeout = param.value
                    self.get_logger().info('set aruco_timeout success')
                else:
                    self.get_logger().warn('aruco_pose_timeout value is negative -> skipping')
            else:
                self.get_logger().warn(f"can't set {param.name}")

        return SetParametersResult(successful=True)

    def _control_loop_timer(self):
        if self.state == ControlState.IDLE:
            self.get_logger().info('... idle')
            self.stop()
            return

        with self.control_loop_lock:
            if self.state == ControlState.SCAN_ARUCO:
                self.get_logger().info('... scan around')
                self.scan_around()
                if self.is_found_aruco:
                    self.get_logger().info('!!! found aruco')
                    self.state = ControlState.FOLLOW_ARUCO
                else:
                    self.get_logger().debug('no aruco')
            elif self.state == ControlState.FOLLOW_ARUCO:
                self.get_logger().info('... follow aruco')
                self.follow_aruco()
                if self.is_aruco_timeout:
                    self.get_logger().info('!!! aruco timeout')
                    self.state = ControlState.SCAN_ARUCO

        self.get_logger().info('\n\n\n')

    def set_active_handle(self, request:SetBool.Request, response:SetBool.Response):
        self.get_logger().info(f'set_active to : {request.data}')

        response.success = True
        response.message = f'active from state:{self.state.name}'

        if request.data:
            self.state = ControlState.SCAN_ARUCO
        else:
            self.state = ControlState.IDLE

        return response

    def odometry_cb(self, msg:Odometry):
        # Tutor Debug [Odom Frame is Ignored]
        empty_odom = Odometry()
        empty_odom.pose.pose.orientation.w = 1.0
        # end Tutor Debug [Odom Frame is Ignored]

        self.current_pose = empty_odom.pose.pose #msg.pose.pose
        self.get_logger().debug(f'odom_cb : {self.current_pose}')

    def aruco_pose_cb(self, msg:PoseWithCovarianceStamped):
        """
        Pose with Covariance Stamped Marker Detection
        """
        self.lastest_aruco_stamped = self.get_clock().now().nanoseconds*TO_SEC
        self.aruco_pose = PoseStamped()
        self.aruco_pose.header.stamp = msg.header.stamp
        self.aruco_pose.header.frame_id = msg.header.frame_id #camera_frame
        self.aruco_pose.pose.position = msg.pose.pose.position
        self.aruco_pose.pose.orientation = msg.pose.pose.orientation
        # self.get_logger().debug('aruco_pose_cb')

    def scan_around(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = self.scan_around_vel

        self.pub_cmd_vel(cmd_vel)

    def follow_aruco(self):
        goal_pose = self.calculate_goal_pose()
        try:
            if goal_pose:
                self.get_logger().debug(f'{self.current_pose}')
                self.get_logger().debug(f'{goal_pose}')
                # Calculate the Velocity
                cmd_vel = self.go_to_pose(self.current_pose, goal_pose)
                # Publish Velocity
                self.pub_cmd_vel(cmd_vel)
            else:
                self.get_logger().warn('goal_pose is none')
        except Exception as e:
            self.get_logger().warn('{e}')

    def calculate_goal_pose(self):
        try:
            tf_aruco_odom = self.tf_buffer.lookup_transform(
                self.robot_base_frame, self.aruco_frame,
                Time(seconds=0),
            )
            goal_pose = Pose()
            goal_pose.position.x = tf_aruco_odom.transform.translation.x
            goal_pose.position.y = tf_aruco_odom.transform.translation.y
            goal_pose.position.z = tf_aruco_odom.transform.translation.z
            goal_pose.orientation.x = tf_aruco_odom.transform.rotation.x
            goal_pose.orientation.y = tf_aruco_odom.transform.rotation.y
            goal_pose.orientation.z = tf_aruco_odom.transform.rotation.z
            goal_pose.orientation.w = tf_aruco_odom.transform.rotation.w

            frontal_dist = tf_aruco_odom.transform.translation.x - self.goal_pose_offset
            # Offset Calculation to prevent negative goal
            if frontal_dist < 0.0:
                frontal_dist = 0.0

            goal_pose.position.x = frontal_dist

            self.publish_goal_pose(goal_pose)
            return goal_pose

        except Exception as e:
            self.get_logger().warn(f'{e}')
            return None

    def publish_goal_pose(self, pose:Pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.robot_base_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        self.goal_vis_pub.publish(pose_stamped)

    def stop(self):
        self.pub_cmd_vel(Twist())

    def pub_cmd_vel(self, cmd_vel:Twist):
        self.cmd_vel_pub.publish(self.clamped_vel(cmd_vel))

    def clamped_vel(self, cmd_vel:Twist) -> Twist:
        clamped_cmd_vel = Twist()
        clamped_cmd_vel.linear.x = min(max(cmd_vel.linear.x, self.min_linear_vel), self.max_linear_vel)
        clamped_cmd_vel.angular.z = min(max(abs(cmd_vel.angular.z), self.min_angular_vel), self.max_angular_vel) * np.sign(cmd_vel.angular.z)

        return clamped_cmd_vel

def main(args=None):
    rclpy.init(args=args)

    controller = RobotController()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)

    try:
        executor.spin()
    except Exception as e:
        print(e)
        controller.destroy_node()
    finally:
        try:
            executor.shutdown(0)
            rclpy.shutdown()
        except:
            print('rclpy already shutdown')

if __name__=="__main__":
    main()