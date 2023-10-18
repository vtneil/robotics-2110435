import rclpy
from rclpy.node import Node
import std_msgs.msg
import geometry_msgs.msg
import turtlesim.msg
import time
import math

NAME = "turtle_square"


class TurtleSquareNode(Node):
    TARG_ANG = [1 / 4 * math.pi,
                math.pi,
                -1 / 2 * math.pi,
                2 * math.pi - 0.001,
                1 / 2 * math.pi]
    TARG_POS = [((9, 9), 0.1),
                ((1, 9), 0.1),
                ((1, 1), 0.1),
                ((9, 1), 0.1),
                ((9, 9), 0.5)]

    def __init__(self, name: str):
        super().__init__(name)

        self.time_start = time.time()
        self.counter = 0
        self.state = 0
        self.TIM_PERIOD = 0.1
        self.pos: turtlesim.msg.Pose = turtlesim.msg.Pose()
        self.twist: geometry_msgs.msg.Twist = geometry_msgs.msg.Twist()

        self.turtle_pub = self.create_publisher(geometry_msgs.msg.Twist,
                                                '/turtle1/cmd_vel', 10)
        self.pos_sub = self.create_subscription(turtlesim.msg.Pose,
                                                '/turtle1/pose',
                                                self._sub_callback, 10)
        self.timer = self.create_timer(self.TIM_PERIOD, self._timer_callback)

    def _eval_state(self):
        if self.counter >= min(len(self.TARG_ANG), len(self.TARG_POS)):
            return

        self._logger.info(f'State is {self.counter} : {self.state}.')
        if self.state == 0:
            now = self.normalize(self.pos.theta)
            targ = self.normalize(self.TARG_ANG[self.counter])
            d = targ - now
            if not abs(d) <= math.radians(1):
                if d >= 0:
                    d = max(1 * math.pi / 180, d)
                else:
                    d = min(-1 * math.pi / 180, d)
                self.twist = self.make_twist(0, 0, 0,
                                             0, 0, d)
            else:
                self.twist = self.make_twist()
                self.state = 1

        elif self.state == 1:
            now = (self.pos.x, self.pos.y)
            targ = self.TARG_POS[self.counter][0]
            d = math.dist(now, targ)
            if not d < self.TARG_POS[self.counter][1]:
                d = max(0.1, d)
                self.twist = self.make_twist(d, 0, 0,
                                             0, 0, 0)
            else:
                self.twist = self.make_twist()
                self.state = 0
                self.counter += 1

    def _timer_callback(self):
        self._logger.info(f'Position is ({self.pos.x:.3f}, {self.pos.y:.3f}, {self.pos.theta:.3f})')
        self._eval_state()
        self._pub_vel(self.twist)
        self._logger.info(f'Published {self.twist} at {self.millis:.2f}')

    def _sub_callback(self, msg):
        self.pos = msg

    def _pub_vel(self, msg: geometry_msgs.msg.Twist):
        self.turtle_pub.publish(msg)

    @staticmethod
    def normalize(t):
        return t % math.tau

    @staticmethod
    def make_twist(x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0):
        return geometry_msgs.msg.Twist(**{
            'linear': geometry_msgs.msg.Vector3(**{
                'x': float(x),
                'y': float(y),
                'z': float(z)
            }),
            'angular': geometry_msgs.msg.Vector3(**{
                'x': float(rx),
                'y': float(ry),
                'z': float(rz)
            })
        })

    @property
    def millis(self):
        return time.time() - self.time_start


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSquareNode(NAME)
    rclpy.spin(node)
    rclpy.shutdown()
