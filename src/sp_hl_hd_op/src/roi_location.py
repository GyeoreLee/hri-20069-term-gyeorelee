#!/usr/bin/env python
# get ROI and find distance (x,y,z) from depth image
# Create: 2016-04-04 by qvnguyen@ksit.re.kr
#
# To run this program using rasbag file: you have to convert compressed + compressedDepth images to raw iamges, as follow: 
# rosrun image_transport republish compressedDepth in:=/_kinect1/depth/image_raw raw out:=/sn_kinect1/depth/image_raw
# rosrun image_transport republish compressed in:=/_kinect1/rgb/image_color raw out:=/sn_kinect1/rgb/image_color

#import roslib
#roslib.load_manifest('human_detector_cnn')
import roslib
#import roslib.packages
roslib.load_manifest('sp_hl_hd_op')
from roslib import scriptutil
import sys, select, termios, tty
import sys, os, time
import rospy
from time import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32MultiArray
from numpy import *
import numpy as np
from threading import Lock
from pub_msgs.msg import where_msgs
from pub_msgs.msg import roilocation_msgs
from geometry_msgs.msg import Point32
import matplotlib.pyplot as plt
import scipy.io as sio
import argparse
import scipy
import matplotlib.pyplot as plt
import cv2
import cv
from math import * 
import math

where_mutex = Lock()
fusion_mutex = Lock()
class HumanDetectorRGBD:
	def __init__(self):
		self.init_variable()
#sub and pub toptics
		rospy.init_node('republish_depth2_roi', anonymous=False)
		self.depth_sub1 = rospy.Subscriber("/depth_mask/sn_kinect2", roilocation_msgs, self.depth_roi_mask_callback1, queue_size = 1)
		self.depth_sub5 = rospy.Subscriber("/sn_kinect2/depth/image_raw2", Image, self.depth_callback1, queue_size = 1)
		
		self.image_pub = rospy.Publisher("sn_kinect2/depth/image_raw_roi",Image, queue_size = 1)
#		rospy.Timer(rospy.Duration(0.001), self.timer_callback)
	def init_variable(self):
########################
		self.bridge = CvBridge()
		self.depth_kinect1 = None
		self.depth_roi_mask_kinect1=None
		self.depthkinect1_ready=0
		self.depth_roi_mask_kinect1_ready=0		
	def depth_roi_mask_callback1(self,data):
		fusion_mutex.acquire()
		self.depth_roi_mask_kinect1_ready=1

#		index=0
#		if len(data.cam_id)>0:
#			for i in range(480):
#				for j in range(640):
#					if data.roi_location[index]==1:
#						self.depth_roi_mask_image1[i,j]=1
#					index=index+1
		self.depth_roi_mask_kinect1=arange(480*640)
		self.depth_roi_mask_kinect1.shape=(480,640)
		self.depth_roi_mask_kinect1[:,:]=0				
		if len(data.cam_id)>0:
			for n in  xrange(0,len(data.roi_location)-1,2):
				 self.depth_roi_mask_kinect1[data.roi_location[n],data.roi_location[n+1]]=1
		fusion_mutex.release()
	def depth_callback1(self,data):
		where_mutex.acquire()
		self.depth_kinect1 = self.bridge.imgmsg_to_cv2(data, "16UC1")
		self.depthkinect1_ready=1
		if self.depth_roi_mask_kinect1_ready==1:
			self.depth_kinect1[self.depth_roi_mask_kinect1 ==0] = 0
#			cv_image_mask = np.array(self.depth_kinect1, dtype=np.float32)
#			cv2.normalize(cv_image_mask, cv_image_mask, 0, 1, cv2.NORM_MINMAX)
#			small = cv2.resize(cv_image_mask, (0,0), fx=0.6, fy=0.6)                     
#			cv2.imshow('Tracking ROI 1: ', small)
#			cv2.waitKey(3)

		self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.depth_kinect1, "16UC1"))
		where_mutex.release()
def main(args):
	ic = HumanDetectorRGBD()    
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv2.destroyAllWindows()
if __name__ == '__main__':
	main(sys.argv)
