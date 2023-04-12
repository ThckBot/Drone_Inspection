#!/usr/bin/env python

# Run using rosrun drone_control t265_publisher_node.py
import rospy
from std_msgs.msg import Point


rospy.init_node('THCK_depth_publisher')
pub = rospy.Publisher('/THCK_depth_publisher_topic', Point, queue_size=10)

rate = rospy.Rate(10)  # Publish at 10 Hz

while not rospy.is_shutdown():
    x = 1.0
    y = 2.0

    # TODO set x=x and y=y to the obstacles and the depths respectively
    my_point = Point(x=x, y=y)
    pub.publish(my_point)
    rate.sleep()