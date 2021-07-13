#!/usr/bin/python3
from __future__ import print_function
from artigo_framework.srv import CameraUAV, CameraUAVResponse
import rospy
import numpy as np
import matplotlib.pyplot as plt
import message_filters

from sensor_msgs.msg import Range, Image, CameraInfo
from std_msgs.msg import String, Int32
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


import cv2
import os
from cv_bridge import CvBridge

############################ Functions ############################

def stationary_camera_transform(point, alturaZ ):
    K = [215.6810060961547, 0.0, 376.5, 0.0, 215.6810060961547, 240.5, 0.0, 0.0, 1.0]

    baseTerrestreAltura = 0.000009 # 0.5

    Z = alturaZ - baseTerrestreAltura # Distancia do solo

    dX = (point[0] - K[2]) * Z / K[0]
    dY = (point[1] - K[5]) * Z / K[4]

    dist = (dX, dY)
    return dist

def algae_detector(img):
    #print(img)
    img = img[:,:,[0,1,2]]
    img_ch0 = img[:,:,0]
#thr
    min_thr = 20
    _, thr = cv2.threshold(img_ch0, min_thr, 255, cv2.THRESH_BINARY)
#blur
    size_b = 5
    kernel_blur = np.ones((size_b, size_b),np.float32)/size_b*size_b
    dst = cv2.filter2D(thr, -1, kernel_blur)
#closing
    size_c = 10
    kernel_closing = np.ones((size_c, size_c),np.float32)/size_c*size_c
    img_binary = cv2.morphologyEx(dst, cv2.MORPH_CLOSE, kernel_closing)

    mostTrue = thr.shape[0]*thr.shape[1]*255*0.98 # 98% of the image's pixels have the same color (255)
    mostFalse = thr.shape[0]*thr.shape[1]*255*0.02    #  2% of the image's pixels have the same color (255)
    img_pixels = thr.shape[0]*thr.shape[1]
    if (thr.sum() >= mostTrue) or (thr.sum() <= mostFalse):
        #print("Is the world blue?")
        B_sum = img[:,:,2].sum()
        G_sum = img[:,:,1].sum()
        R_sum = img[:,:,0].sum()

        red_limit = 60
        blue_limit = 150
        if (R_sum < red_limit*thr.shape[0]*thr.shape[1]) and (B_sum > blue_limit*thr.shape[0]*thr.shape[1]) :
            #print("yes, it is!")
            img_binary = np.zeros(thr.shape)

    img2 = cv2.cvtColor(img_binary, cv2.COLOR_GRAY2RGB)
    #print(img2.shape)
    contours, hierarchy = cv2.findContours(img_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #_, contours, _ = cv2.findContours(img2, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
    centres = []
    if len(contours)>0:
        for i in range(len(contours)):
            moments = cv2.moments(contours[i])
            centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
    
    return centres

################################ Service #######################################
def handle_goals(req):
    # print(req)
    image_ros = req.image_sub
    location = req.GPS_sub
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_ros, desired_encoding='passthrough')
    classified = algae_detector(image)
    uav_z = location.pose.pose.position.z
    uav_x = location.pose.pose.position.x
    uav_y = location.pose.pose.position.y
    #pub.publish(bridge.cv2_to_imgmsg(classified))
    x_list = []
    y_list = []
    for i in classified:
        p = stationary_camera_transform((i[0], 480-i[1]), uav_z)
        algae_coord = (round(uav_x+p[0]+116,1), round(uav_y+p[1]+120, 1) )
        x_list.append(algae_coord[0])
        y_list.append(algae_coord[1])
        # print(algae_coord)
        # print(x_list, y_list)

    return CameraUAVResponse(x_list, y_list)

def add_two_ints():
	rospy.init_node('algae_to_coord_server') #### Create server node

### Create service: the first argument is the service name, the second the srv declared on line 3 for variables, the last is the callback
	s = rospy.Service('algae_to_coord', CameraUAV, handle_goals)
	#print("Ready to row.")
	rospy.spin()

if __name__ == "__main__":
	add_two_ints()
