import time
from math import pi, tau, atan2, sqrt

from .robot_helper import *
from nav_msgs.msg import Odometry


class Params:
    NAME = "aruco_explorer"


class State(Enum):
    IDLE = 0
    WALK_GOAL = 1
    SEARCH_ARUCO = 2
    FOLLOW_ARUCO = 3
    HALT = -1


class ArucoExplorerNode(Node):
    TIM_PERIOD = 0.1

    THRESHOLD_ANGLE = 2 * (pi / 180)  # rad
    SEARCH_SPEED = 15 * (pi / 180)

    THRESHOLD_DISTANCE = 0.01  # meter
    DISTANCE_CAL_SCALE = 0.5  # 0.5 m real world =  1 m value
    TARG_DIST_ARUCO = 0.40 / DISTANCE_CAL_SCALE
    TARG_ANG_ARUCO = 0

    MAX_VEL = 0.306  # From iRobot Create 3 Data
    MAX_ROT = 45 * (pi / 180)

    ARUCO_TIMEOUT = 20  # times the delta t of previous aruco

    def __init__(self, name: str):
        super().__init__(name)
        
        # todo
        raise NotImplementedError('Not Done!')

        self.time_start: float = time.time()
        self.__time_aruco: float = 0.0
        self.__last_aruco: float = 0.0

        self.marked: list = []

        self.motion_state: StateMachine = StateMachine(start=State.IDLE)
        self.assign_states(self.motion_state)

        self.twist: Twist = make_twist()

        self.aruco_pos: PoseWithCovarianceStamped = PoseWithCovarianceStamped()
        self.odom: Odometry = Odometry()
        # self.robot_pos

        self.pid_pos = Controller.PID(1, 0, 0)
        self.pid_rot = Controller.PID(1, 0, 0)

        self.pub_motion = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_aruco = self.create_subscription(PoseWithCovarianceStamped,
                                                  '/aruco_detector/marker',
                                                  self.sub_aruco_callback, 10)

        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)

        self.sub_odom = self.create_subscription(Odometry,
                                                 '/odom',
                                                 self.sub_odom_callback, 10)

        self.timer = self.create_timer(self.TIM_PERIOD, self.timer_callback)
        self.timer_print = self.create_timer(0.25, self.timer_print_callback)

    def timer_print_callback(self):
        self._logger.info(f'Evaluated state is {self.motion_state.state.name} ({self.motion_state.state.value}).')
        self._logger.info(f'Published the following at {self.millis():.2f} sec. since power.')
        self._logger.info(f'        ( X  ,  Y  ,  Z  )')
        self._logger.info(f'Linear  ({self.twist.linear.x:.2f}, {self.twist.linear.y:.2f}, {self.twist.linear.z:.2f})')
        self._logger.info(
            f'Angular ({self.twist.angular.x:.2f}, {self.twist.angular.y:.2f}, {self.twist.angular.z:.2f})')
        self._logger.info('=' * 40)

    def timer_callback(self):
        self.logic_controller()
        self.pub_motion_pub(self.twist)

    def pub_motion_pub(self, msg: Twist):
        # Forward function
        self.pub_motion.publish(msg)

    def pub_odom_pub(self, msg: Odometry):
        # Forward function
        self.pub_odom.publish(msg)

    def sub_aruco_callback(self, msg: PoseWithCovarianceStamped):
        self.aruco_pos = msg
        self.__last_aruco = self.__millis_aruco()
        self.__time_aruco = self.millis()

    def sub_odom_callback(self, msg: Odometry):
        self.odom = msg

    def logic_controller(self):
        # todo
        self.motion_state.execute()

    def assign_states(self, state_machine: StateMachine):
        def state_idle():
            self.twist = make_twist()
            state_machine.state = State.SEARCH_ARUCO

        def state_search():
            if self.__aruco_timeout():
                self.twist = make_twist(rz=self.SEARCH_SPEED)
                state_machine.state = State.SEARCH_ARUCO
            else:
                self.pid_pos.send((self.millis(),
                                   self.TARG_DIST_ARUCO,
                                   self.TARG_DIST_ARUCO))
                self.pid_rot.send((self.millis(),
                                   self.TARG_ANG_ARUCO,
                                   self.TARG_ANG_ARUCO))
                state_machine.state = State.FOLLOW_ARUCO

        def state_follow():
            if not self.__aruco_timeout():
                out_pos = self.pid_pos.send((self.millis(),
                                             self.aruco_pos.pose.pose.position.z,
                                             self.TARG_DIST_ARUCO))
                diff_theta = atan2(self.aruco_pos.pose.pose.position.x, 5.0)
                out_rot = self.pid_rot.send((self.millis(),
                                             diff_theta,
                                             self.TARG_ANG_ARUCO))
                act_pos = -sgn(out_pos) * max(abs(out_pos), abs(self.MAX_VEL))
                act_rot = sgn(out_rot) * max(abs(out_rot), abs(self.MAX_ROT))
                self.twist = make_twist(act_pos, 0, 0,
                                        0, 0, act_rot)
            else:
                self.twist = make_twist()
                state_machine.state = State.SEARCH_ARUCO

        state_machine.set_callbacks((State.IDLE, state_idle),
                                    (State.SEARCH_ARUCO, state_search),
                                    (State.FOLLOW_ARUCO, state_follow))

    def millis(self):
        return time.time() - self.time_start

    def __millis_aruco(self):
        return self.millis() - self.__time_aruco

    def __aruco_timeout(self) -> bool:
        return self.__millis_aruco() > (self.ARUCO_TIMEOUT * self.__last_aruco)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoExplorerNode(Params.NAME)
    rclpy.spin(node)
    rclpy.shutdown()
