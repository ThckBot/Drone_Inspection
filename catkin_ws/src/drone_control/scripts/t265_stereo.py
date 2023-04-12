#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.
# Python 2/3 compatibility
from __future__ import print_function

"""
This example shows how to use T265 intrinsics and extrinsics in OpenCV to
asynchronously compute depth maps from T265 fisheye images on the host.
T265 is not a depth camera and the quality of passive-only depth options will
always be limited compared to (e.g.) the D4XX series cameras. However, T265 does
have two global shutter cameras in a stereo configuration, and in this example
we show how to set up OpenCV to undistort the images and compute stereo depth
from them.
Getting started with python3, OpenCV and T265 on Ubuntu 16.04:
First, set up the virtual enviroment:
$ apt-get install python3-venv  # install python3 built in venv support
$ python3 -m venv py3librs      # create a virtual environment in pylibrs
$ source py3librs/bin/activate  # activate the venv, do this from every terminal
$ pip install opencv-python     # install opencv 4.1 in the venv
$ pip install pyrealsense2      # install librealsense python bindings
Then, for every new terminal:
$ source py3librs/bin/activate  # Activate the virtual environment
$ python3 t265_stereo.py        # Run the example
"""

# First import the library
#import pyrealsense2 as rs

# Import OpenCV and numpy
import cv2
import numpy as np
from math import tan, pi
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import time
import matplotlib.pyplot as plt

data_left, data_right = None, None
K_left, D_left, R_left, P_left = None, None, None, None
K_right, D_right, R_right, P_right = None, None, None, None


"""
In this section, we will set up the functions that will translate the camera
intrinsics and extrinsics from librealsense into parameters that can be used
with OpenCV.
The T265 uses very wide angle lenses, so the distortion is modeled using a four
parameter distortion model known as Kanalla-Brandt. OpenCV supports this
distortion model in their "fisheye" module, more details can be found here:
https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html
"""

"""
Returns R, T transform from src to dst
"""
def get_extrinsics(src, dst):
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3,3]).T
    T = np.array(extrinsics.translation)
    return (R, T)

"""
Returns a camera matrix K from librealsense intrinsics
"""
def camera_matrix(intrinsics):
    return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                     [            0, intrinsics.fy, intrinsics.ppy],
                     [            0,             0,              1]])

"""
Returns the fisheye distortion from librealsense intrinsics
"""
def fisheye_distortion(intrinsics):
    return np.array(intrinsics.coeffs[:4])



"""
This callback is called on a separate thread, so we must use a mutex
to ensure that data is synchronized properly. We should also be
careful not to do much work on this thread to avoid data backing up in the
callback queue.
"""
def callback1(data):
    global data_left
    bridge = CvBridge()

    data_left = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

def callback2(data):
    global data_right
    bridge = CvBridge()

    data_right = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

def callback_intrinsics_1(data):
    global K_left, D_left, R_left, P_left
    K_left = np.array(data.K).reshape(3,3)
    D_left = np.array(data.D)
    R_left = np.array(data.R).reshape(3,3)
    P_left = np.array(data.P).reshape(3,4)


def callback_intrinsics_2(data):
    global K_right, D_right, R_right, P_right
    K_right = np.array(data.K).reshape(3,3)
    D_right = np.array(data.D)
    R_right = np.array(data.R).reshape(3,3)
    P_right = np.array(data.P).reshape(3,4)

try:
    rospy.init_node('drone_stereo', anonymous=True)
    # Set up an OpenCV window to visualize the results
    WINDOW_TITLE = 'Realsense'
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)

    # Configure the OpenCV stereo algorithm. See
    # https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html for a
    # description of the parameters
    window_size = 5
    min_disp = 0
    # must be divisible by 16
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp
    stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
                                   numDisparities = num_disp,
                                   blockSize = 11,
                                   P1 = 16*3*window_size**2,
                                   P2 = 128*3*window_size**2,
                                #    preFilterCap = 63,
                                   disp12MaxDiff = 1,
                                   uniquenessRatio = 5,
                                   speckleWindowSize = 50,
                                   speckleRange = 16)

    # Retreive the stream and intrinsic properties for both cameras
    #profiles = pipe.get_active_profile()
    
    fisheye1 = rospy.Subscriber('/camera/fisheye1/image_raw', Image, callback1)
    fisheye2 = rospy.Subscriber('/camera/fisheye2/image_raw', Image, callback2)
    intrinsics1 = rospy.Subscriber('/camera/fisheye1/camera_info', CameraInfo, callback_intrinsics_1)
    intrinsics2 = rospy.Subscriber('/camera/fisheye2/camera_info', CameraInfo, callback_intrinsics_2)

    # We need to determine what focal length our undistorted images should have
    # in order to set up the camera matrices for initUndistortRectifyMap.  We
    # could use stereoRectify, but here we show how to derive these projection
    # matrices from the calibration and a desired height and field of view

    # We calculate the undistorted focal length:
    #
    #         h
    # -----------------
    #  \      |      /
    #    \    | f  /
    #     \   |   /
    #      \ fov /
    #        \|/
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

    # wait for all subscribers to initialize
    print('waiting...')
    time.sleep(5)

    print('done waiting, init undistort')

    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
    (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
    undistort_rectify = {"left"  : (lm1, lm2),
                         "right" : (rm1, rm2)}

    mode = "stack"
    while True:
        valid = False
        time.sleep(0.5)
        valid = True
        print('Valid')

        # If frames are ready to process
        if valid:
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
            depth_masked = np.ma.masked_inside(depth,0.8,2)
            depth_mask = depth_masked.mask
            print(np.argwhere(depth_mask))
            depth[~depth_mask] = 0
            mid_col = np.argmax(np.sum(depth_mask,axis=0))
            obs_depth = np.average(depth[:,mid_col][depth_mask[:,mid_col]])
            print('Middle column is:', mid_col)
            print('Depth is:', obs_depth)

            # convert disparity to 0-255 and color it
            disp_vis = 255*(disparity - min_disp)/ num_disp
            disp_color = cv2.applyColorMap(cv2.convertScaleAbs(disp_vis,1), cv2.COLORMAP_JET)
            color_image = cv2.cvtColor(center_undistorted["left"][:,max_disp:], cv2.COLOR_GRAY2RGB)
            if mode == "stack":
                #cv2.imshow(WINDOW_TITLE, np.hstack((color_image, depth)))
                plt.imshow(depth, vmin=0, vmax = 5)
                plt.colorbar()
                plt.show()
            if mode == "overlay":
                ind = disparity >= min_disp
                color_image[ind, 0] = disp_color[ind, 0]
                color_image[ind, 1] = disp_color[ind, 1]
                color_image[ind, 2] = disp_color[ind, 2]
                cv2.imshow(WINDOW_TITLE, color_image)
        key = cv2.waitKey(0)
        key = ord('s')
        if key == ord('s'): mode = "stack"
        if key == ord('o'): mode = "overlay"
        if key == ord('q') or cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
            break
except rospy.ROSInterruptException:	
    print("didnt make it in stereo node")	
    pass