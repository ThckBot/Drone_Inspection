#!/usr/bin/env python

from __future__ import print_function

# Import OpenCV and numpy
import cv2
import numpy as np
from math import tan, pi
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import time
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

data_left, data_right = None, None
K_left, D_left, R_left, P_left = None, None, None, None
K_right, D_right, R_right, P_right = None, None, None, None

depth_requested = False

class t265_stereo:
    def __init__(self):
        rospy.init_node('THCK_depth_publisher')
        self.pub = rospy.Publisher('/THCK_depth_response_topic', Point, queue_size=1)
        self.sub = rospy.Subscriber('/THCK_depth_request_topic', Bool, self.depth_request_callback, queue_size=1)
        self.rate = rospy.Rate(10)  # Publish at 10 Hz
        self.depth_requested = False

    def depth_request_callback(self,bool_msg):
        self.depth_requested = bool_msg
    def get_extrinsics(self,src, dst):
        extrinsics = src.get_extrinsics_to(dst)
        R = np.reshape(extrinsics.rotation, [3,3]).T
        T = np.array(extrinsics.translation)
        return (R, T)

    def camera_matrix(self,intrinsics):
        return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                        [            0, intrinsics.fy, intrinsics.ppy],
                        [            0,             0,              1]])

    def fisheye_distortion(self,intrinsics):
        return np.array(intrinsics.coeffs[:4])
    
    def callback1(self,data):
        global data_left
        bridge = CvBridge()

        data_left = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def callback2(self,data):
        global data_right
        bridge = CvBridge()

        data_right = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def callback_intrinsics_1(self,data):
        global K_left, D_left, R_left, P_left
        K_left = np.array(data.K).reshape(3,3)
        D_left = np.array(data.D)
        R_left = np.array(data.R).reshape(3,3)
        P_left = np.array(data.P).reshape(3,4)


    def callback_intrinsics_2(self,data):
        global K_right, D_right, R_right, P_right
        K_right = np.array(data.K).reshape(3,3)
        D_right = np.array(data.D)
        R_right = np.array(data.R).reshape(3,3)
        P_right = np.array(data.P).reshape(3,4)

    def gen_obs_coord(self):

        # Set up an OpenCV window to visualize the results
        # WINDOW_TITLE = 'Realsense'
        # cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)

        # Configure the OpenCV stereo algorithm. 
        window_size = 5
        min_disp = 0
        num_disp = 112 - min_disp
        max_disp = min_disp + num_disp
        stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
                                    numDisparities = num_disp,
                                    blockSize = 11,
                                    P1 = 16*3*window_size**2,
                                    P2 = 128*3*window_size**2,
                                    disp12MaxDiff = 1,
                                    uniquenessRatio = 5,
                                    speckleWindowSize = 50,
                                    speckleRange = 16)

        # Retreive the stream and intrinsic properties for both cameras        
        fisheye1 = rospy.Subscriber('/camera/fisheye1/image_raw', Image, self.callback1)
        fisheye2 = rospy.Subscriber('/camera/fisheye2/image_raw', Image, self.callback2)
        intrinsics1 = rospy.Subscriber('/camera/fisheye1/camera_info', CameraInfo, self.callback_intrinsics_1)
        intrinsics2 = rospy.Subscriber('/camera/fisheye2/camera_info', CameraInfo, self.callback_intrinsics_2)

        stereo_fov_rad = 90 * (pi/180)  # 90 degree desired fov
        stereo_height_px = 500          # 300x300 pixel stereo output
        stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)

        # We set the left rotation to identity and the right rotation
        # the rotation between the cameras

        # The stereo algorithm needs max_disp extra pixels in order to produce valid
        # disparity on the desired output region. This changes the width, but the
        # center of projection should be on the center of the cropped image
        stereo_width_px = stereo_height_px + max_disp
        stereo_size = (stereo_width_px, stereo_height_px)
        stereo_cx = (stereo_height_px - 1)/2 + max_disp
        stereo_cy = (stereo_height_px - 1)/2

        # Construct the left and right projection matrices, the only difference is
        # that the right projection matrix should have a shift along the x axis of
        # baseline*focal_length

        # Create an undistortion map for the left and right camera which applies the
        # rectification and undoes the camera distortion. This only has to be done
        # once
        m1type = cv2.CV_32FC1
        time.sleep(1)

        #print('done waiting, init undistort')

        (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
        (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
        undistort_rectify = {"left"  : (lm1, lm2),
                            "right" : (rm1, rm2)}


        print('Frame copy')
        # Hold the mutex only long enough to copy the stereo frames
        frame_copy = {"left"  : data_left,
                    "right" : data_right}

        # Undistort and crop the center of the frames
        center_undistorted = {"left" : cv2.remap(src = frame_copy["left"],
                                    map1 = undistort_rectify["left"][0],
                                    map2 = undistort_rectify["left"][1],
                                    interpolation = cv2.INTER_LINEAR),
                            "right" : cv2.remap(src = frame_copy["right"],
                                    map1 = undistort_rectify["right"][0],
                                    map2 = undistort_rectify["right"][1],
                                    interpolation = cv2.INTER_LINEAR)}

        # compute the disparity on the center of the frames and convert it to a pixel disparity (divide by DISP_SCALE=16)
        disparity = stereo.compute(center_undistorted["left"], center_undistorted["right"]).astype(np.float32) / 16.0

        # re-crop just the valid part of the disparity
        disparity = disparity[:,max_disp:]
        depth = (K_left[0,0]*0.064) / (disparity + 1e-6) 
        depth_masked = np.ma.masked_inside(depth,1.2,4.5)
        depth_mask = depth_masked.mask
        depth[~depth_mask] = 0
        mid_col = np.argmax(np.sum(depth_mask,axis=0))
        obs_depth = np.average(depth[0:300,mid_col][depth_mask[0:300,mid_col]])
        print("================")
        print('Middle column is:', mid_col)
        print('Depth is:', obs_depth)
        #print("Image shape", depth_mask.shape)
        coords = Point()
        #stereo_cx = 250
        #print("Cx is: ", stereo_cx)
        coords.x = (mid_col-stereo_cx)*obs_depth/stereo_focal_px
        print('X is: ', coords.x)
        coords.y = 0
        coords.z = obs_depth
        self.pub.publish(coords)
        self.rate.sleep()


if __name__ == '__main__':
    print('waiting...')
    stereo = t265_stereo()
    while not rospy.is_shutdown():
        if stereo.depth_requested:
            stereo.gen_obs_coord()
            stereo.depth_requested = False
        else:
            stereo.rate.sleep()
