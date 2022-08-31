#!/usr/bin/env python

"""Example of a ROS 2 node where the 'control loop' is actually
handled in a loop rather than through a timer callback.

This pattern was common in ROS 1, but requires some ugly hackery in
ROS 2 because of the different default threading model.

Note that this approach raises the danger of race conditions since
callbacks may happen during the execution of the main loop.

Author: Nathan Sprague
Version: 8/31/2022

"""
import rclpy
import rclpy.node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

class ThrusterNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('thruster')

        # ReentrantCallbackGroups allow for multiple callbacks
        # to execute simultaneously.  This is not the default!
        group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.location = None # Stores recently received location messages.

        self.thrust_pub = self.create_publisher(Vector3, 'thrust', 10)

        # This timer exists only to fire off the first call to the
        # run method. The run method will destroy the timer
        # to prevent it from happening more than once.
        self.timer = self.create_timer(.1, self.run,
                                       callback_group=group)

        self.create_subscription(Point, 'location',
                                 self.location_callback, 10,
                                 callback_group=group)

        self.target_altitude = 100.0

    def location_callback(self, loc_msg):
        """loc_msg will be of type Point."""
        self.location = loc_msg

    def run(self):
        """The main loop for this node."""
        self.destroy_timer(self.timer)

        rate = self.create_rate(10.0)

        while rclpy.ok():

            if self.location is not None:
                thrust = Vector3()
                if self.location.y < self.target_altitude:
                    thrust.y = 100.0
                    self.get_logger().info("THRUSTERS ENGAGED")
                else:
                    thrust.y = 0.0
                self.thrust_pub.publish(thrust)

            rate.sleep()  # Sleep long enough to maintain the desired rate.


def main():
    rclpy.init()
    thruster_node = ThrusterNode()

    # We need at least two threads: one for the main loop, one for the
    # location callback.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(thruster_node)
    executor.spin()

    thruster_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
