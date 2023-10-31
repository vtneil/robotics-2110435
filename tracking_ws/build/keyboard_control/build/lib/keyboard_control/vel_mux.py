 #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

class VelMux(Node):
    def __init__(self):
        super().__init__('vel_mux')
        self.mode = False
        self.park_mode = False

        # Create a subscriber
        self.manual_subscription = self.create_subscription(
            Twist,
            'manaul/cmd_vel',
            self.manual_subscription_callback,
            1
        )
        self.auto_subscription = self.create_subscription(
            Twist,
            'auto/cmd_vel',
            self.auto_subscription_callback,
            1
        )

        # Create a publisher
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        # Create a service server
        self.service = self.create_service(
            SetBool,
            '/toAuto_mode',
            self.service_callback
        )
        self.park_service = self.create_service(
            SetBool,
            '/toPark_mode',
            self.park_service_callback
        )

    def manual_subscription_callback(self, msg):
        if (self.mode == False) and (self.park_mode == True):
            self.publisher.publish(msg)

    def auto_subscription_callback(self, msg):
        if (self.mode == True) and (self.park_mode == True):
            self.publisher.publish(msg)

    def service_callback(self, request, response):
        # self.get_logger().info(f"Service request: {request.data}")
        self.mode = request.data
        response.success = True
        if (self.mode == False) and (self.park_mode == True):
            self.get_logger().info("Switch to mode: Manual Moving")
        elif (self.mode == True) and (self.park_mode == True):
            self.get_logger().info("Switch to mode: Auto Moving")
        elif self.park_mode == False:
            self.get_logger().info("You're in parking mode, please switch to Moving mode (press p) if you would like to Manual or Auto moving :)")
        return response
    
    def park_service_callback(self, request, response):
        # self.get_logger().info(f"Service request: {request.data}")
        self.park_mode = request.data
        response.success = True
        if self.park_mode == False:
            self.get_logger().info("Switch to mode: Parking")
        else:
            self.get_logger().info("Switch to mode: Moving")
        return response

def main(args=None):
    rclpy.init(args=args)
    vel_mux = VelMux()
    rclpy.spin(vel_mux)
    vel_mux.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
