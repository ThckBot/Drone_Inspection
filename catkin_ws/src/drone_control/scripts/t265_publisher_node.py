#!/usr/bin/env python

# Run using rosrun drone_control t265_publisher_node.py
import rospy
from std_msgs.msg import Point, Bool


depth_requested = False

def depth_request_callback(bool_msg):
    depth_requested = bool_msg

rospy.init_node('THCK_depth_publisher')
pub = rospy.Publisher('/THCK_depth_response_topic', Point, queue_size=10)
sub = rospy.Subscriber('/THCK_depth_request_topic', Bool, depth__request_callback, queue_size=10)

rate = rospy.Rate(10)  # Publish at 10 Hz

while not rospy.is_shutdown():
    x = 1.0
    y = 2.0

    # TODO set x=x and y=y to the obstacles and the depths respectively
    my_point = Point(x=x, y=y)
    pub.publish(my_point)
    rate.sleep()