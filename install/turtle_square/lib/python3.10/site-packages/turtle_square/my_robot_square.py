import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


def normalize_angle(theta_raw):
    # Normalize Raw Theta to [-pi, pi]
    if theta_raw > math.pi:
        theta_raw -= 2 * math.pi
    elif theta_raw < -math.pi:
        theta_raw += 2 * math.pi
    return theta_raw


class TurtleSquareStateMachine(Node):
    def __init__(self):
        super().__init__('turtle_square_node')
        self.states = {}  # Dictionary to store state transitions
        self.current_state = None  # The current state of the machine

        # Targets
        self.targets = [(1, 1), (1, 9), (9, 9), (9, 1), (1, 1)]

        # Variables
        self.current_pose: Pose = None
        self.current_target_idx = 0

        # Create Subscriber for turtle pose
        self.pose_subscriber = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'turtle1/cmd_vel',
            10)

        self.get_logger().info("initialize completed!!")

    def pose_callback(self, msg):
        self.current_pose = msg

    def process(self):
        if self.current_state is None:
            raise ValueError("Initial state not set")

        state_function = self.states[self.current_state]
        next_state = state_function(self)

        if next_state is not None:
            self.transition_to(next_state)

    def run(self):
        # Create Timer Process Loop
        self.process_timer = self.create_timer(0.1, self.process)

    def exit(self):
        self.get_logger().info('stop stm process')
        self.process_timer.cancel()

    def add_state(self, state_name, state_function):
        self.states[state_name] = state_function

    def set_initial_state(self, state_name):
        if state_name in self.states:
            self.current_state = state_name
        else:
            raise ValueError("State not found")

    def transition_to(self, state_name):
        if state_name in self.states:
            self.current_state = state_name
        else:
            raise ValueError("State not found")


def state_idle(stm: TurtleSquareStateMachine):
    stm.get_logger().info('EXECUTE : [IDLE]')

    try:
        if stm.current_pose and stm.targets[stm.current_target_idx]:
            return 'ROTATE'
        else:
            stm.get_logger().warning("current_pose is None .. fallback to IDLE")
            return 'IDLE'
    except IndexError:
        stm.exit()
        stm.get_logger().info("Draw Square Succeed!!")
        return 'IDLE'


def state_rotate(stm: TurtleSquareStateMachine):
    stm.get_logger().info('EXECUTE : [ROTATE]')
    current_goal = stm.targets[stm.current_target_idx]

    stm.get_logger().info("Turning to {}".format(current_goal))

    if not stm.current_pose:
        stm.get_logger().warning("current_pose is None .. fallback to IDLE")
        return "IDLE"

    x = current_goal[0] - stm.current_pose.x
    y = current_goal[1] - stm.current_pose.y
    current_theta = stm.current_pose.theta
    angle_remain = math.atan2(y, x) - current_theta

    stm.get_logger().info("Remaining Angle : {:.2f}".format(angle_remain))
    if abs(angle_remain) <= 0.02:
        return "WALK"
    else:
        # Calculate Angular Velocity
        angular_velocity = 0.3 * angle_remain
        # Publish Twist Message
        msg = Twist()
        msg.angular.z = angular_velocity
        stm.cmd_vel_publisher.publish(msg)
        return "ROTATE"


def state_walk(stm: TurtleSquareStateMachine):
    stm.get_logger().info('EXECUTE : [WALK]')
    current_goal = stm.targets[stm.current_target_idx]

    stm.get_logger().info("Walking to {}".format(current_goal))

    if not stm.current_pose:
        stm.get_logger().warning("current_pose is None .. fallback to IDLE")
        return "IDLE"

    x = current_goal[0] - stm.current_pose.x
    y = current_goal[1] - stm.current_pose.y
    current_theta = stm.current_pose.theta

    distance_remain = math.sqrt(x ** 2 + y ** 2)
    angle_remain = normalize_angle(math.atan2(y, x) - current_theta)

    stm.get_logger().info("Remaining Angle : {:.2f}".format(angle_remain))
    stm.get_logger().info("Remaining Distance : {:.2f}".format(distance_remain))
    if 0.0 <= distance_remain <= 0.15:
        return "ADVANCE_GOAL"
    else:
        # Calculate Velocity
        angular_velocity = 0.2 * angle_remain
        linear_velocity = 0.4 * distance_remain
        # Publish Twist Message
        msg = Twist()
        msg.angular.z = angular_velocity
        msg.linear.x = linear_velocity
        stm.cmd_vel_publisher.publish(msg)
        return "WALK"


def state_advance_goal(stm: TurtleSquareStateMachine):
    stm.get_logger().info('EXECUTE : [ADVANCE_GOAL]')

    stm.current_target_idx += 1
    if stm.current_target_idx >= len(stm.targets):
        return "IDLE"
    else:
        return "ROTATE"


def main():
    rclpy.init()
    stm = TurtleSquareStateMachine()
    stm.add_state("IDLE", state_idle)
    stm.add_state("ROTATE", state_rotate)
    stm.add_state("WALK", state_walk)
    stm.add_state("ADVANCE_GOAL", state_advance_goal)
    stm.set_initial_state("IDLE")
    stm.run()

    try:
        rclpy.spin(stm)
    finally:
        try:
            stm.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
