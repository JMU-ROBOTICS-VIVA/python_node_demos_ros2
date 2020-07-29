#!/usr/bin/env python

"""Python class illustrating a style of ROS node with no explicit main
 loop.  The logic is completely event driven.

This node doesn't store incoming sensor messages.  Instead it responds
to them directly from the callback.

Author: Nathan Sprague
Version: 9/8/2015

"""
import rclpy
import rclpy.node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

class ThrusterNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('thruster')
        
        self.create_subscription(Point, 'location',
                                 self.location_callback, 10)
        self.thrust_pub = self.create_publisher(Vector3, 'thrust', 10)

        self.target_altitude = 100.0

    def location_callback(self, loc_msg):
        """ loc_msg will be of type Point """
        thrust = Vector3()

        if loc_msg.y < self.target_altitude:
            thrust.y = 100.0
            self.get_logger().info("THRUSTERS ENGAGED")
        else:
            thrust.y = 0.0

        self.thrust_pub.publish(thrust)

if __name__ == "__main__":
    rclpy.init()
    thruster_node = ThrusterNode()
    rclpy.spin(thruster_node)

    thruster_node.destroy_node()
    rclpy.shutdown()

