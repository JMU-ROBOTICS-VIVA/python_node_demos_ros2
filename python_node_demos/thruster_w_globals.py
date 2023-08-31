#!/usr/bin/env python

"""Example of a ROS2 node that uses globals to store the
values received from sensor callbacks.  Not very good style.

Author: Nathan Sprague
Version: 8/31/2023

"""
import rclpy

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

# globals
location = None
publisher = None
node = None
TARGET_ALTITUDE = 100


# This function will be called every time a new location message arrives.
def location_callback(loc_msg):
    """loc_msg will be of type Point."""

    global location  # prevents creation of a local location variable.
    location = loc_msg


def timer_callback():

    if location is not None:
        thrust = Vector3()

        if location.y < TARGET_ALTITUDE:
            thrust.y = 100.0
            node.get_logger().info("THRUSTERS ENGAGED")
        else:
            thrust.y = 0.0

        publisher.publish(thrust)


def main(args=None):
    """Initialize the node."""

    # Switch on ROS communication.
    rclpy.init(args=args)

    # Create a node object that will be used to manage ROS communication.
    global node
    node = rclpy.create_node('thruster')

    global publisher
    publisher = node.create_publisher(Vector3, 'thrust', 10)
    node.create_timer(.1, timer_callback)

    # Subscribe to the location topic.
    node.create_subscription(Point, 'location', location_callback, 10)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
