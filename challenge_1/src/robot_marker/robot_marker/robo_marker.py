import time
from math import pi, tau, atan2, sqrt
from enum import Enum
from typing import Callable

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class Params:
    NAME = "aruco_follower"


class State(Enum):
    IDLE = 0
    SEARCH = 1
    FOLLOW = 2


class StateMachine:
    def __init__(self, start: State):
        self.__state: State = start
        self.__func_map: dict = {k: None for k in State}
        self.__en: bool = True

    def enable(self):
        self.__en = True

    def disable(self):
        self.__en = False

    def evaluate(self, *args, **kwargs):
        fn = self.__func_map[self.__state]
        if fn is not None:
            return fn(*args, **kwargs)

    def set_func(self, state: State, func: Callable):
        if not isinstance(state, State):
            return
        self.__func_map[state] = func

    def set_callback(self, state: State, func: Callable):
        self.set_func(state, func)

    @property
    def state(self):
        return self.__state

    @state.setter
    def state(self, new_state: State):
        if isinstance(new_state, State):
            self.__state = new_state


class PID:
    @staticmethod
    def controller(k_p, k_i, k_d, mv_bar=0):
        # initialize stored data
        e_prev = 0
        t_prev = 0
        i = 0

        # initial control
        mv = mv_bar

        while True:
            t, pv, sp = yield mv

            e = sp - pv

            p = k_p * e
            i = i + k_i * e * (t - t_prev)
            d = k_d * (e - e_prev) / (t - t_prev)

            mv = mv_bar + p + i + d

            e_prev = e
            t_prev = t


def sgn(x):
    return 1 if x >= 0 else -1


class ArucoFollowerNode(Node):
    TIM_PERIOD = 0.1

    THRESHOLD_ANGLE = 2 * (pi / 180)  # rad
    SEARCH_SPEED = 45 * (pi / 180)

    THRESHOLD_DISTANCE = 0.01  # meter
    DISTANCE_CAL_SCALE = 0.5  # 0.5 m real world =  1 m value
    TARG_DIST_ARUCO = 0.20 / DISTANCE_CAL_SCALE
    TARG_ANG_ARUCO = 0

    MAX_VEL = 0.306
    MAX_ROT = 45 * (pi / 180)

    def __init__(self, name: str):
        super().__init__(name)

        self.time_start = time.time()
        self.__time_aruco = 0
        self.__last_aruco = 0

        self.counter = 0

        self.state_machine = StateMachine(start=State.IDLE)
        self.assign_states()

        self.twist: Twist = self.make_twist()
        self.aruco_pos: PoseWithCovarianceStamped = PoseWithCovarianceStamped()
        self.pid_pos = PID.controller(0.5, 0, 0)
        self.pid_pos.send(None)
        self.pid_rot = PID.controller(0.5, 0, 0)
        self.pid_rot.send(None)

        self.pub_motion = self.create_publisher(Twist,
                                                '/cmd_vel', 10)

        self.sub_aruco = self.create_subscription(PoseWithCovarianceStamped,
                                                  '/aruco_detector/marker',
                                                  self.sub_callback, 10)

        self.timer = self.create_timer(self.TIM_PERIOD, self.timer_callback)

    def timer_callback(self):
        # self._logger.info(f'Position is ({self.pos.x:.3f}, {self.pos.y:.3f}, {self.pos.theta:.3f})')

        self.state_machine.evaluate()
        self.pub_vel(self.twist)

        self._logger.info(f'Evaluated state is {self.state_machine.state}.')
        self._logger.info(f'Published the following at {self.millis():.2f} sec. since power.')
        self._logger.info(f'        ( X  ,  Y  ,  Z  )')
        self._logger.info(f'Linear  ({self.twist.linear.x:.2f}, {self.twist.linear.y:.2f}, {self.twist.linear.z:.2f})')
        self._logger.info(f'Angular ({self.twist.angular.x:.2f}, {self.twist.angular.y:.2f}, {self.twist.angular.z:.2f})')
        self._logger.info('=' * 40)

    def sub_callback(self, msg):
        self.aruco_pos = msg
        self.__last_aruco = self.millis_aruco()
        self.__time_aruco = self.millis()

    def pub_vel(self, msg: Twist):
        self.pub_motion.publish(msg)

    def assign_states(self):
        def state_idle():
            self.twist = self.make_twist()
            self.state_machine.state = State.SEARCH

        def state_search():
            if self.aruco_timeout():
                self.twist = self.make_twist(0, 0, 0,
                                             0, 0, self.SEARCH_SPEED)
                self.state_machine.state = State.SEARCH
            else:
                self.pid_pos.send((self.millis(),
                                   self.TARG_DIST_ARUCO,
                                   self.TARG_DIST_ARUCO))
                self.pid_rot.send((self.millis(),
                                   self.TARG_ANG_ARUCO,
                                   self.TARG_ANG_ARUCO))
                self.state_machine.state = State.FOLLOW

        def state_follow():
            if self.aruco_timeout():
                out_pos = self.pid_pos.send((self.millis(),
                                             self.aruco_pos.pose.pose.position.z,
                                             self.TARG_DIST_ARUCO))
                diff_theta = atan2(self.aruco_pos.pose.pose.position.x,
                                   self.aruco_pos.pose.pose.position.z)
                out_rot = self.pid_rot.send((self.millis(),
                                             diff_theta,
                                             self.TARG_ANG_ARUCO))
                act_pos = sgn(out_pos) * max(abs(out_pos), abs(self.MAX_VEL))
                act_rot = sgn(out_rot) * max(abs(out_rot), abs(self.MAX_ROT))
                self.twist = self.make_twist(act_pos, 0, 0,
                                             0, 0, act_rot)
            else:
                self.twist = self.make_twist()
                self.state_machine.state = State.SEARCH

        self.state_machine.set_callback(State.IDLE, state_idle)
        self.state_machine.set_callback(State.SEARCH, state_search)
        self.state_machine.set_callback(State.FOLLOW, state_follow)

    @staticmethod
    def normalize(t):
        return t % tau

    @staticmethod
    def make_twist(x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0):
        return Twist(**{
            'linear': Vector3(**{
                'x': float(x),
                'y': float(y),
                'z': float(z)
            }),
            'angular': Vector3(**{
                'x': float(rx),
                'y': float(ry),
                'z': float(rz)
            })
        })

    def millis(self):
        return time.time() - self.time_start

    def millis_aruco(self):
        return self.millis() - self.__time_aruco

    def aruco_timeout(self) -> bool:
        return self.millis_aruco() > (2 * self.__last_aruco)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollowerNode(Params.NAME)
    rclpy.spin(node)
    rclpy.shutdown()
