import dataclasses
import math
from typing import Callable, TypeVar, Generic

from enum import Enum

import rclpy
from rclpy.node import Node

import geometry_msgs
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

StateT = TypeVar('StateT', bound=Enum)


class StateMachine(Generic[StateT]):
    def __init__(self, start: StateT):
        self.__state: StateT = start
        self.func_map: dict = dict()
        # self.__func_map: dict = {k: None for k in StateT}

    def execute(self, *args, **kwargs):
        if self.__state in self.func_map:
            fn = self.func_map[self.__state]
        else:
            return
        if fn is not None:
            return fn(*args, **kwargs)

    def __call__(self, *args, **kwargs):
        return self.execute(*args, **kwargs)

    def set_func(self, state: StateT, func: Callable):
        self.func_map[state] = func

    def set_callback(self, state: StateT, func: Callable):
        self.set_func(state, func)
        return self

    def set_callbacks(self, *args: tuple[StateT, Callable] | list[StateT, Callable]):
        for arg in args:
            if len(arg) != 2:
                raise IndexError('Length of arguments must be 2: StateT and Callable')
            self.set_callback(arg[0], arg[1])

    @property
    def state(self):
        return self.__state

    @state.setter
    def state(self, new_state: StateT):
        self.__state = new_state


class Controller:
    class PID:
        def __init__(self, k_p, k_i, k_d, mv_bar=0):
            self.k_p = k_p
            self.k_i = k_i
            self.k_d = k_d
            self.mv_bar = mv_bar

            self.e_prev = 0
            self.t_prev = 0
            self.i = 0
            self.mv = self.mv_bar

            self._controller = self._make_controller()
            next(self._controller)

        def send(self, args):
            return self._controller.send(args)

        def __call__(self, *args):
            return self.send(args)

        def _make_controller(self):
            while True:
                args = yield self.mv
                if args:
                    t, pv, sp = args

                    e = sp - pv

                    p = self.k_p * e
                    self.i = self.i + self.k_i * e * (t - self.t_prev)
                    d = self.k_d * (e - self.e_prev) / (t - self.t_prev)

                    self.mv = self.mv_bar + p + self.i + d

                    self.e_prev = e
                    self.t_prev = t

        @property
        def controller(self):
            return self._controller

    class P:
        def __init__(self, k_p, mv_bar=0):
            self.k_p = k_p
            self.mv_bar = mv_bar
            self.mv = self.mv_bar

            self._controller = self._make_controller()
            next(self._controller)

        def send(self, args):
            return self._controller.send(args)

        def _make_controller(self):
            while True:
                args = yield self.mv
                if args:
                    pv, sp = args
                    e = sp - pv
                    p = self.k_p * e
                    self.mv = self.mv_bar + p

        @property
        def controller(self):
            return self._controller


def sgn(x):
    return 1 if x >= 0 else -1


def normalize(angle):
    return angle % math.tau


@dataclasses.dataclass
class MyQuaternion:
    x: float | int = None
    y: float | int = None
    z: float | int = None
    w: float | int = None

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z
        yield self.w

    def to_ros2(self):
        return geometry_msgs.msg.Quaternion(
            x=self.x,
            y=self.y,
            z=self.z,
            w=self.w,
        )

    @staticmethod
    def from_ros2(quaternion: geometry_msgs.msg.Quaternion):
        return MyQuaternion(
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w
        )


@dataclasses.dataclass
class MyEuler:
    x: float | int = None
    y: float | int = None
    z: float | int = None

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z


def quaternion_to_euler(quaternion: MyQuaternion):
    x, y, z, w = quaternion

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return MyEuler(roll_x, pitch_y, yaw_z)


def euler_to_quaternion(euler: MyEuler):
    roll, pitch, yaw = euler

    qx = (math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) -
          math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2))
    qy = (math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) +
          math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2))
    qz = (math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) -
          math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2))
    qw = (math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) +
          math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2))

    return MyQuaternion(qx, qy, qz, qw)


quaternion_from_euler = euler_to_quaternion
euler_from_quaternion = quaternion_to_euler


def get_next_coordinate(x0, y0,
                        xm, zm,
                        heading_ref):
    x = x0 + (zm * math.cos(heading_ref) - xm * math.sin(heading_ref))
    y = y0 + (zm * math.sin(heading_ref) - xm * math.cos(heading_ref))
    return x, y


def staticvar(**kwargs):
    raise DeprecationWarning('Deprecated')
    # def decorate(func):
    #     for k in kwargs:
    #         setattr(func, k, kwargs[k])
    #     return func

    # return decorate


def make_twist(x=0.0, y=0.0, z=0.0,
               rx=0.0, ry=0.0, rz=0.0):
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


def dist(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def max_mag(a, b):
    return sgn(a) * max(abs(a), abs(b))


def min_mag(a, b):
    return sgn(a) * min(abs(a), abs(b))


__all__ = [
    'math',
    'StateMachine',
    'Controller',
    'sgn',
    'normalize',
    'MyEuler',
    'MyQuaternion',
    'quaternion_to_euler',
    'euler_from_quaternion',
    'euler_to_quaternion',
    'quaternion_from_euler',
    'Enum',
    'rclpy',
    'Node',
    'PoseWithCovarianceStamped',
    'Twist',
    'Vector3',
    'staticvar',
    'make_twist',
    'get_next_coordinate',
    'dist',
    'max_mag'
]
