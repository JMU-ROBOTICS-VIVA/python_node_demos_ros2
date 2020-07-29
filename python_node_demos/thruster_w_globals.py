#!/usr/bin/env python

"""Example of a ROS node that uses globals to store the
values received from sensor callbacks.  Not very good style.

Author: Nathan Sprague
Version: 7/22/2020

"""
import rclpy

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

# globals
LOCATION = None
PUBLISHER = None
NODE = None
TARGET_ALTITUDE = 100

# This function will be called every time a new location message arrives.
def location_callback(loc_msg):
    """ loc_msg will be of type Point """

    global LOCATION # prevents creation of a local LOCATION variable.
    LOCATION = loc_msg


def timer_callback():
    
    if LOCATION is not None:
        thrust = Vector3()
        
        if LOCATION.y < TARGET_ALTITUDE:
            thrust.y = 100.0
            NODE.get_logger().info("THRUSTERS ENGAGED")
        else:
            thrust.y = 0.0
            
        PUBLISHER.publish(thrust)

    
def main():
    """ Initialize the node """

    # Switch on ROS communication.
    rclpy.init()

    # Create a node object that will be used to manage ROS communication.
    global NODE
    NODE = rclpy.create_node('thruster')
    
    global PUBLISHER
    PUBLISHER = NODE.create_publisher(Vector3, 'thrust', 10)
    NODE.create_timer(.1, timer_callback)
    
    # Subscribe to the location topic.
    NODE.create_subscription(Point, 'location', location_callback, 10)


    rclpy.spin(NODE)


    NODE.destroy_node()
    rclpy.shutdown()


# This is how we usually call the main method in Python. 
if __name__ == "__main__":
    main()
