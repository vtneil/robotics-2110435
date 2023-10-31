import dataclasses
import math
from typing import Callable, TypeVar, Generic

StateT = TypeVar('StateT')


class StateMachine(Generic[StateT]):
    def __init__(self, start: StateT):
        self.__state: StateT = start
        self.__func_map: dict = {k: None for k in StateT}

    def execute(self, *args, **kwargs):
        fn = self.__func_map[self.__state]
        if fn is not None:
            return fn(*args, **kwargs)

    def set_func(self, state: StateT, func: Callable):
        if type(state) is StateT:
            return
        self.__func_map[state] = func

    def set_callback(self, state: StateT, func: Callable):
        self.set_func(state, func)

    @property
    def state(self):
        return self.__state

    @state.setter
    def state(self, new_state: StateT):
        if type(new_state) is StateT:
            self.__state = new_state


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


def sgn(x):
    return 1 if x >= 0 else -1


@dataclasses.dataclass
class Quaternion:
    x: float | int = None
    y: float | int = None
    z: float | int = None
    w: float | int = None

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z
        yield self.w


@dataclasses.dataclass
class Euler:
    x: float | int = None
    y: float | int = None
    z: float | int = None

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z


def quaternion_to_euler(quaternion: Quaternion):
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

    return Euler(roll_x, pitch_y, yaw_z)


def euler_to_quaternion(euler: Euler):
    roll, pitch, yaw = euler

    qx = (math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) -
          math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2))
    qy = (math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) +
          math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2))
    qz = (math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) -
          math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2))
    qw = (math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) +
          math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2))

    return Quaternion(qx, qy, qz, qw)


__all__ = [
    'StateMachine',
    'PID',
    'sgn',
    'Euler',
    'Quaternion',
    'quaternion_to_euler',
    'euler_to_quaternion'
]
