import time
from math import pi, tau, atan2, sqrt

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose

from .robot_helper import *
from rclpy.action import ActionClient

from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from irobot_create_msgs.action import NavigateToPosition
from aruco_msgs.msg import Marker, MarkerArray


class Params:
    NAME = "aruco_explorer"


class State(Enum):
    IDLE = 0
    WALK_GOAL = 1
    SEARCH_ARUCO = 2
    FOLLOW_ARUCO = 3
    HALT = -1
    WAIT = -2


class ArucoExplorerNode(Node):
    TICK_HZ = 10
    TIM_PERIOD = 1 / TICK_HZ

    THRESHOLD_ANGLE = math.radians(2)  # rad
    SEARCH_SPEED = math.radians(15)

    THRESHOLD_DISTANCE = 0.05  # meter
    DISTANCE_CAL_SCALE = 0.5  # 0.5 m real world =  1 m value
    TARG_DIST_ARUCO = 0.30 / DISTANCE_CAL_SCALE
    TARG_ANG_ARUCO = 0

    MAX_VEL = 0.306  # From iRobot Create 3 Data
    MAX_ROT = 45 * (pi / 180)

    ARUCO_TIMEOUT = 20  # times the delta t of previous aruco

    robot_controls = [
        (State.SEARCH_ARUCO, 0),  # Marker 0
        (State.WALK_GOAL, (-1., 0.)),  # Position to search Marker 4

        (State.SEARCH_ARUCO, 4),  # Marker 4
        (State.WALK_GOAL, (0., 0.)),  # Position to search Marker 5

        (State.SEARCH_ARUCO, 5),  # Marker 5
        (State.WALK_GOAL, (0., 0.))  # Between 4 and 5 (Dynamic)
    ]

    def __init__(self, name: str):
        super().__init__(name)

        self.time_start: float = time.time()
        self.__time_aruco: float = 0.0
        self.__last_aruco: float = 0.0

        self.marked: list = []

        self.motion_state: StateMachine = StateMachine(start=State.IDLE)
        self.assign_states()

        self.twist: Twist = make_twist()

        self.aruco_markers: list[Marker] = []
        self.aruco_markers_id: list[int] = []
        self.odom: Odometry = Odometry()
        self.robot_pos: Pose = self.odom.pose.pose

        self.marker_pos: dict[int, tuple[float, float]] = {}

        self.pid_pos = Controller.PID(0.75, 0, 0)
        self.pid_rot = Controller.PID(0.50, 0, 0)

        self.control_iter = iter(self.robot_controls)
        self.control_now = None
        self.togg: bool = False

        self.action_client = ActionClient(self, NavigateToPosition, '/navigate_to_position')

        self.pub_motion = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_aruco = self.create_subscription(MarkerArray,
                                                  '/aruco_detector/marker_array',
                                                  self.sub_aruco_callback, 10)

        __pub_odom = self.create_publisher(Odometry,
                                           '/odom',
                                           10)

        for _ in range(5):
            __pub_odom.publish(Odometry())

        self.sub_odom = self.create_subscription(Odometry,
                                                 '/odom',
                                                 self.sub_odom_callback, 10)

        self.pub_speaker = self.create_publisher(AudioNoteVector,
                                                 '/cmd_audio',
                                                 10)

        self.timer = self.create_timer(self.TIM_PERIOD, self.timer_callback)
        self.timer_print = self.create_timer(self.TIM_PERIOD or 0.500, self.timer_print_callback)

    def timer_print_callback(self):
        self._logger.info(f'Evaluated state is {self.motion_state.state.name} ({self.motion_state.state.value}).')
        self._logger.info(f'Control status at {self.control_now}.')
        self._logger.info(f'Markers: {self.aruco_markers_id}')
        self._logger.info(
            f'Odometry Position: ({self.robot_pos.position.x}, '
            f'{self.robot_pos.position.y}, '
            f'{self.robot_pos.position.z})'
        )
        self._logger.info(
            f'Odometry Orientation: ({self.robot_pos.orientation.w}, {self.robot_pos.orientation.x}, '
            f'{self.robot_pos.orientation.y}, {self.robot_pos.orientation.z})'
        )

        self._logger.info(f'Found Markers Position:')
        for m in self.marker_pos:
            self._logger.info(f'ID: {m} = {self.marker_pos[m]}')

        self._logger.info(f'Published the following at {self.millis():.2f} sec. since program time.')
        self._logger.info(f'        ( X  ,  Y  ,  Z  )')
        self._logger.info(f'Linear  ({self.twist.linear.x:.2f}, {self.twist.linear.y:.2f}, {self.twist.linear.z:.2f})')
        self._logger.info(
            f'Angular ({self.twist.angular.x:.2f}, {self.twist.angular.y:.2f}, {self.twist.angular.z:.2f})')
        self._logger.info('=' * 40)

    def timer_callback(self):
        self.logic_controller()

    def pub_motion_pub(self, msg: Twist):
        # Forward function
        self.pub_motion.publish(msg)

    def pub_speaker_beep(self):
        rtime = Duration(sec=1, nanosec=0)
        note = AudioNote(frequency=100, max_runtime=rtime)
        msg = AudioNoteVector(append=False, notes=[note])
        self.pub_speaker.publish(msg)

    def sub_aruco_callback(self, msg: MarkerArray):
        self.aruco_markers = msg.markers
        self.aruco_markers_id = [e.id for e in self.aruco_markers]
        if self.control_now is not None and \
                self.control_now[0] in [State.SEARCH_ARUCO, State.FOLLOW_ARUCO] and \
                self.control_now[1] in self.aruco_markers_id:
            self.__last_aruco = self.__millis_aruco()
            self.__time_aruco = self.millis()

    def sub_odom_callback(self, msg: Odometry):
        self.odom = msg
        self.robot_pos = self.odom.pose.pose

    def logic_controller(self):
        if self.motion_state.state == State.IDLE:
            try:
                self.control_now = next(self.control_iter)
                self.togg = True
            except StopIteration:
                self.control_now = (State.HALT, None)

        if self.togg:
            if self.control_now[0] == State.WALK_GOAL:
                self.motion_state.state = State.WALK_GOAL
                self.togg = False

            elif self.control_now[0] == State.SEARCH_ARUCO:
                self.motion_state.state = State.SEARCH_ARUCO
                self.togg = False

            elif self.control_now[0] == State.HALT:
                self.togg = False
                raise StopIteration('Control has ended.')

        self.motion_state()
        self.pub_motion_pub(self.twist)

    def assign_states(self):
        def state_idle():
            self.twist = make_twist()

        def state_search():
            if self.aruco_is_timeout():
                self.twist = make_twist(rz=self.SEARCH_SPEED)
                self.motion_state.state = State.SEARCH_ARUCO
            else:
                # Reset PID
                self.pid_pos(self.millis(),
                             self.TARG_DIST_ARUCO,
                             self.TARG_DIST_ARUCO)
                self.pid_rot(self.millis(),
                             self.TARG_ANG_ARUCO,
                             self.TARG_ANG_ARUCO)
                if self.control_now[1] in self.aruco_markers_id:
                    for i in range(5):
                        self._logger.warn('FOUND FOUND FOUND FOUND FOUND')
                    self.motion_state.state = State.FOLLOW_ARUCO

        def state_follow():
            if not self.aruco_is_timeout():
                for marker in self.aruco_markers:
                    if marker.id == self.control_now[1]:
                        if abs(marker.pose.pose.position.z - self.TARG_DIST_ARUCO) <= self.THRESHOLD_DISTANCE:
                            self.motion_state.state = State.IDLE
                            self.twist = make_twist()

                            if marker.id in [4, 5]:
                                # Update Marker Position

                                x0 = self.robot_pos.position.x
                                y0 = self.robot_pos.position.y
                                robot_rot = euler_from_quaternion(MyQuaternion.from_ros2(self.robot_pos.orientation))
                                t_ref = normalize(robot_rot.z)
                                (xm, zm) = (marker.pose.pose.position.x, marker.pose.pose.position.z)
                                x = x0 + (zm * math.cos(t_ref) - xm * math.sin(t_ref))
                                y = y0 + (zm * math.sin(t_ref) - xm * math.cos(t_ref))
                                this_marker_pos = (x, y)
                                self.marker_pos[marker.id] = this_marker_pos
                                if 4 in self.marker_pos and 5 in self.marker_pos:
                                    p45 = (0.5 * (self.marker_pos[4][0] + self.marker_pos[5][0]),
                                           0.5 * (self.marker_pos[4][1] + self.marker_pos[5][1]))
                                    self.robot_controls[-1] = (State.WALK_GOAL, p45)

                            # Exit if camera pointed at the marker at appropriate position
                            return

                        out_pos = self.pid_pos(self.millis(),
                                               marker.pose.pose.position.z,
                                               self.TARG_DIST_ARUCO)
                        diff_theta = atan2(marker.pose.pose.position.x, 5.0)
                        out_rot = self.pid_rot(self.millis(),
                                               diff_theta,
                                               self.TARG_ANG_ARUCO)
                        act_pos = -sgn(out_pos) * max(abs(out_pos), abs(self.MAX_VEL))
                        act_rot = sgn(out_rot) * max(abs(out_rot), abs(self.MAX_ROT))
                        self.twist = make_twist(act_pos, 0, 0,
                                                0, 0, act_rot)
            else:
                self.twist = make_twist()
                self.motion_state.state = State.SEARCH_ARUCO

        def state_walk_goal():
            d = dist(self.robot_pos.position.x, self.robot_pos.position.y,
                     self.control_now[1][0], self.control_now[1][1])

            if d <= self.THRESHOLD_DISTANCE:
                self.motion_state.state = State.IDLE
                self.twist = make_twist()
                return

            d_targ = 0
            out_pos = self.pid_pos(self.millis(), d, d_targ)

            d_theta = atan2(self.control_now[1][0] - self.robot_pos.position.x,
                            self.control_now[1][1] - self.robot_pos.position.y)
            t_targ = 0
            out_rot = self.pid_rot(self.millis(), d_theta, t_targ)

            act_pos = max_mag(out_pos, self.MAX_VEL)
            act_rot = max_mag(out_rot, self.MAX_ROT)

            self.twist = make_twist(x=act_pos, rz=act_rot)

        self.motion_state.set_callbacks((State.IDLE, state_idle),
                                        (State.SEARCH_ARUCO, state_search),
                                        (State.FOLLOW_ARUCO, state_follow),
                                        (State.WALK_GOAL, state_walk_goal),
                                        (State.HALT, None))

    def millis(self):
        return time.time() - self.time_start

    def __millis_aruco(self):
        return self.millis() - self.__time_aruco

    def aruco_is_timeout(self) -> bool:
        logic = self.__millis_aruco() > (self.ARUCO_TIMEOUT * self.__last_aruco)
        return logic


def main(args=None):
    rclpy.init(args=args)
    node = ArucoExplorerNode(Params.NAME)
    rclpy.spin(node)
    rclpy.shutdown()
