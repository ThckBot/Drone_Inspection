import numpy as np
import rospy
from drone_fsm import *

from geometry_msgs.msg import PoseStamped,TransformStamped, Point
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State 
from sensor_msgs.msg import Image
import cv2

# Obstacle detection node
def obstacle_detect():

    local_pos = rospy.Subscriber('/mavros/local_position/odom', Odometry, local_position_callback, queue_size=10) # publishes both position and orientation (quaternion)
    rospy.Subscriber('/camera/fisheye1/image_raw', Image, callback)


    return True


if __name__ == '__main__':	
    try:	
        obstacle_detect()	
    except rospy.ROSInterruptException:	
        print("didnt make it in comm_node")	
        pass