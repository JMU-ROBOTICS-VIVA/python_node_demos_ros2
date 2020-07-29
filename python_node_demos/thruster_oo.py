#!/usr/bin/env python

"""Example of a ROS node using Python's OO features.  The node is
represented as a class.  Sensor messages are stored in instance
variables.

Author: Nathan Sprague
Version: 1/15/2019

"""
import rclpy
import rclpy.node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

class ThrusterNode(rclpy.node.Node):
    
    def __init__(self):
        super().__init__('thruster')

        self.location = None # Stores recently received location messages.

        self.thrust_pub = self.create_publisher(Vector3, 'thrust', 10)
        self.create_timer(.1, self.timer_callback)
        
        self.create_subscription(Point, 'location',
                                 self.location_callback, 10)

        self.target_altitude = 100.0

    def location_callback(self, loc_msg):
        """ loc_msg will be of type Point """
        self.location = loc_msg

    def timer_callback(self):
        if self.location is not None:
            thrust = Vector3()
            if self.location.y < self.target_altitude:
                thrust.y = 100.0
                self.get_logger().info("THRUSTERS ENGAGED")
            else:
                thrust.y = 0.0
            self.thrust_pub.publish(thrust)

def main():
    rclpy.init()
    thruster_node = ThrusterNode()
    rclpy.spin(thruster_node)

    thruster_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
