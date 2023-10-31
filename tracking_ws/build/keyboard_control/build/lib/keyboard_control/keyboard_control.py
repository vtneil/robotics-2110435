 #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
from std_srvs.srv import SetBool

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_vel = self.create_publisher(Twist, 'manaul/cmd_vel', 1)
        self.park_buffer = False
        self.pressed_keys = ''
        self.linear_gain = 0.5
        self.angular_gain = 0.5
        self.lock = threading.Lock()
        self.cli = self.create_client(SetBool, '/toAuto_mode')
        self.park_cli = self.create_client(SetBool, '/toPark_mode')
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('keyboard_control Initialize !')
        self.get_logger().info("\n")
        self.get_logger().info("Initial mode: Parking")
        self.get_logger().info("How to change mode")
        self.get_logger().info("------------------")
        self.get_logger().info("Press p to switch between Parking/Moving")
        self.get_logger().info("-------THEN-------")
        self.get_logger().info("Press m to switch to manual moving")
        self.get_logger().info("Press o to switch to auto moving")
        self.get_logger().info("Press q to exit")

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def timer_callback(self):
        self.pressed_keys = self.getKey()
        command_msg_vel = Twist()
        if 'q' in self.pressed_keys:
            sys.exit(0)
        elif 'w' in self.pressed_keys:
            command_msg_vel.linear.x = 1.0 * self.linear_gain
        elif 's' in self.pressed_keys:
            command_msg_vel.linear.x = -1.0 * self.linear_gain
        elif 'a' in self.pressed_keys:
            command_msg_vel.angular.z = 1.0 * self.angular_gain
        elif 'd' in self.pressed_keys:
            command_msg_vel.angular.z = -1.0 * self.angular_gain        
        elif 'p' in self.pressed_keys:
            # select parking/moving mode
            setbool_req = SetBool.Request()
            if self.park_buffer == False:
                setbool_req.data = True
                future = self.park_cli.call_async(setbool_req)
                self.park_buffer = True
                future.add_done_callback(self.future_callback_function)
            else:
                setbool_req.data = False
                future = self.park_cli.call_async(setbool_req)
                self.park_buffer = False
                future.add_done_callback(self.future_callback_function)
        elif 'o' in self.pressed_keys:
            # select auto mode
            setbool_req = SetBool.Request()
            setbool_req.data = True
            future = self.cli.call_async(setbool_req)
            future.add_done_callback(self.future_callback_function)
        elif 'm' in self.pressed_keys:
            # select manual mode
            setbool_req = SetBool.Request()
            setbool_req.data = False
            future = self.cli.call_async(setbool_req)
            future.add_done_callback(self.future_callback_function)
        else:
            command_msg_vel = Twist()
        self.publisher_vel.publish(command_msg_vel)
    
    def future_callback_function(self, future):
        result = future.result()

def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()
    rclpy.spin(keyboard_control)
    keyboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()