#!/usr/bin/env python
# # -*- coding: utf-8 -*- #
# roscore

# cd ~/realsense_ws
# . devel/setup.bash
# roslaunch realsense2_camera rs_camera.launch

# python aruco_ros.py

# written by Shang-Wen, Wong
# 2021.4.16


import rospy
from sensor_msgs.msg import Image as msg_Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

i = 0
save_img_path = ""

def imageRGBCallback(data):
    global i
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, data.encoding)
        cv2.imshow('Color', cv_image)
        if cv2.waitKey(1) & 0xFF == ord('s'): # 按s存檔
            i = i + 1
            filename = save_img_path + str(i) + '.jpg'
            cv2.imwrite(filename,cv_image)
            print("save:", filename)

    except CvBridgeError as e:
        print(e)
        return

    
if __name__ == '__main__':

    rospy.init_node("save_img")

    sub_markers = rospy.Subscriber('/camera/color/image_raw', msg_Image, imageRGBCallback)
    
    rospy.spin()