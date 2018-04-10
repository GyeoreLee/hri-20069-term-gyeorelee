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
import roslib.packages
roslib.load_manifest('sp_hl_hd_op')
from roslib import scriptutil
import sys, select, termios, tty
import os
import rospy
from time import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32MultiArray
import numpy as np
from threading import Lock
from pub_msgs.msg import where_msgs
from geometry_msgs.msg import Point32

#import _init_paths
#from fast_rcnn.config import cfg
#from fast_rcnn.test import im_detect
#from fast_rcnn.nms_wrapper import nms
#from utils.timer import Timer
import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio
#import caffe, os, sys, cv2
import argparse

import scipy
import matplotlib.pyplot as plt
import cv2
import cv
#import lib for kalman tracking 
import math
import gmphd4 as gmphd
import pickle
where_mutex = Lock()



class HumanDetectorRGBD:
    def __init__(self):
#    	kalman tracking parameter setting
		sigma_q = 0.2
		sigma_r = 0.1
		sigma_p0 = 0.7
		p_d = 0.2
		p_s = 0.99
		merge_thresh = 0.1
		F = [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]]
		Q = [[math.pow(sigma_q, 2), 0, 0, 0], [0, math.pow(sigma_q, 2), 0, 0], [0, 0, 0.001, 0], [0, 0, 0, 0.001]]
		H = [[1, 0, 0, 0], [0, 1, 0, 0]]
		R = [[math.pow(2*sigma_r, 2), 0], [0, math.pow(2*sigma_r, 2)]]
		P0 = [[math.pow(sigma_p0, 2), 0, 0, 0], [0, math.pow(sigma_p0, 2), 0, 0], [0, 0, 0.01, 0], [0, 0, 0, 0.01]]
		clutter_intensity = 0.0
		self.born_components = []
		self.f_gmphd = gmphd.GMPHD([], p_s, p_d, F, Q, H, R, P0, clutter_intensity, merge_thresh)
#sub and pub toptics
		rospy.init_node('human_detection_rgbd_fusion', anonymous=False)
		self.camera = str(rospy.get_param('~camera', 1))
		self.verbose = rospy.get_param('~verbose', True)
		self.sensor_name = 'sn_kinect_' + self.camera
		self.depth_image = None
		self.counter = 0
		
		self.image_pub = rospy.Publisher("/image_topic_"+self.camera, Image, queue_size=1)
		self.where_pub = rospy.Publisher("/sn_kinect/detector_republish", where_msgs, queue_size = 1)
		self.where_pub_tracking = rospy.Publisher("/sn_kinect/detector_tracking", where_msgs, queue_size = 1)
		self.bridge = CvBridge()
		self.where_sub = rospy.Subscriber("/sn_kinect/detector", where_msgs, self.where_callback, queue_size = 1)
		self.depth_sub = rospy.Subscriber("/sn_kinect"+self.camera+"/depth/image_raw2", Image, self.depth_callback, queue_size = 1)
		self.compImage_sub = rospy.Subscriber("/sn_kinect"+self.camera+"/rgb/image_color/compressed", CompressedImage, self.compressedImage_callback,  queue_size = 1)
		plt.ion()
		print 'Start Tracker', self.sensor_name
		self.isBusy = True            
            
    def rgb_callback(self,data):
        self.isBusy = True    
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
          print e

        (rows,cols,channels) = cv_image.shape
        #print 'shape', cv_image.shape
        #print 'max', np.max(cv_image)
        #if cols > 60 and rows > 60 :
        #  cv2.circle(cv_image, (50,50), 10, 255)

        # Detect all object classes and regress object bounds
        self.rgb_image = cv_image
        self.isBusy = False              

    def depth_callback(self,data):

        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError, e:
          print e
        self.new_depth = cv_image  
#        print"test shape", cv_image.shape 
        (rows,cols,channels) = cv_image.shape
#        print"self.counter", self.counter
        if not self.counter:#self.depth_image: 
            self.depth_image = cv_image
        else:            
            self.depth_image = self.new_depth
                                
       
        self.isBusy = False        
        #print 'image', np.max(cv_image), np.min(cv_image)      
        #cv2.imshow('Depth', cv_image*30)        
        #cv2.waitKey(3)
                                      
                  
    def compressedImage_callback(self,data):
        self.isBusy = True    
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(data.data, np.uint8)
        #print 'shape rgb', np_arr.shape        
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        
        self.rgb_image = image_np
        self.isBusy = False           


                            
    def where_callback(self, data):

        if not self.counter:#self.depth_image: 
            self.prev_msg = data
            
        self.counter += 1
        if not self.isBusy:
            cv_image = self.rgb_image            
            font = cv2.FONT_HERSHEY_SIMPLEX  
            (rows,cols,channels) = self.depth_image.shape              
            #publish data in where_msgs
            msg_detect = where_msgs()
            msg_detect.header=data.header
            msg_detect.total=data.total
            msg_detect.cam_id=data.cam_id
            msg_detect.roi=data.roi
            msg_detect.location = []
            msg_detect.user_id = []
            index=0
            for roi in msg_detect.roi:
                #dc = self.depth_image[roi.x_offset+roi.width/2, roi.y_offset+roi.height/2]
                
#                if roi.height > roi.width*1.5:
#                    roi.height = int(roi.width*1.5)
								
                crop = self.depth_image[roi.y_offset+10:roi.y_offset+roi.height-10, roi.x_offset+10:roi.x_offset+roi.width-10]
                x = crop.reshape(-1)
                ind = np.nonzero(x<400)
                ind = np.nonzero(x<100/roi.width*1.5)
                x2 = np.delete(x, ind)
                ind = np.nonzero(x2>5000)
                x3 = np.delete(x2, ind)
                dist = np.median(x3)
                '''
                ahist, bin_edge = np.histogram(x3,100)
                ahist = scipy.signal.medfilt(ahist,kernel_size=3)
                ii = np.argmax(ahist)
                dist = bin_edge[ii]
                '''



                #convert from pixel to x-y-z
                fy = rows / (2 * np.tan(43.2/2*np.pi/180))
                yw = -(rows/2 - (roi.y_offset+roi.height/2)) *dist / fy
                
                fx = cols / (2 * np.tan(57.0/2*np.pi/180))
                xw = (cols/2 - (roi.x_offset+roi.width/2)) *dist / fx
               
                #print 'Person', int(xw), int(yw), int(dist)
                
                #loc = Point32(xw/1000, yw/1000, dist/1000)
                if data.location[index].z==0:
                	loc = Point32(xw/1000, 0.5, dist/1000)
                	msg_detect.user_id.append(-1)
                else:
                	loc = Point32(data.location[index].x, data.location[index].y, data.location[index].z)
                	msg_detect.user_id.append(0)                
                msg_detect.location.append(loc)
                index=index+1
            print "republish message detection",msg_detect.location
            if len(self.prev_msg.roi):
				for roi in self.prev_msg.roi:
					cv2.rectangle(cv_image, (roi.x_offset, roi.y_offset), (roi.x_offset+roi.width, roi.y_offset+roi.height), (100, 255, 255), 1)
				if self.verbose:
					small = cv2.resize(cv_image, (0,0), fx=0.6, fy=0.6)     
                	#self.image_pub.publish(small)        
					cv2.imshow('Distance: '+self.sensor_name, small)
					cv2.waitKey(3)
            self.prev_msg = msg_detect

            self.isBusy = True            
            self.where_pub.publish(msg_detect)         
            #self.depth_image = self.new_depth
            
            
            if 1:#not self.isBusy:
				#cv_image = self.rgb_image
				cv_image = cv2.imread("/home/simonpic/repository/database/GUI/new_gui.png")
				font = cv2.FONT_HERSHEY_SIMPLEX
#				msg_tracking = data
				msg_tracking = where_msgs()
				msg_tracking.header=msg_detect.header
				msg_tracking.total=msg_detect.total
				msg_tracking.cam_id=msg_detect.cam_id
				msg_tracking.roi=msg_detect.roi
				msg_tracking.location=msg_detect.location
				msg_tracking.user_id=msg_detect.user_id
	#			msg.location = []
	#			msg.user_id = []
				pscale = 160
				aa = np.zeros((msg_tracking.total, 2))
				for ii, loc in enumerate(msg_tracking.location):
					if not np.isnan(loc.x):
						aa[ii] = [loc.x, loc.z]
						
						cv2.circle(cv_image, ((int)(loc.x*pscale), (int)(loc.z*pscale-pscale)), 3, (255, 0, 0), 2) 
					print 'Person', loc.x,loc.z
				aa = np.transpose(aa)
				bb=np.zeros((4,aa.shape[1]))
				bb[0]=aa[0]
				bb[1]=aa[1]
				self.f_gmphd.run_iteration(bb)
				if self.verbose:
					gms = self.f_gmphd.gm
					index=0
					for gm in gms:
						if gm.detect:# > 0.1:
							cv2.circle(cv_image, (int(gm.mean[0,0]*pscale), int(gm.mean[1,0]*pscale-pscale)), 3, (0, 255, 0), 2)
							#cv2.rectangle(cv_image, (x, y), (x2, y2), (0, 255, 0), 2)
	#						print"distance after tracking",(gm.mean[0,0]), (gm.mean[1,0])
							msg_tracking.location[msg_tracking.total-1-index].x=(gm.mean[0,0])
							msg_tracking.location[msg_tracking.total-1-index].z=(gm.mean[1,0])
							
							index=index+1
							cv2.putText(cv_image, str(gm.id) +'-{:.2f}'.format(gm.weight), (int(gm.mean[0,0]*pscale), int(gm.mean[1,0]*pscale-pscale)), font ,0.5, (0, 0, 255), 2)
					cv2.imshow('Tracking: '+self.sensor_name, cv_image)
					cv2.waitKey(3)
				
				
				if self.verbose: 
                    #plt.clf()
                    #plt.bar(center, ahist, align='center', width=width)
                    #plt.draw()                                      
                                
                    #print 'len1', len(x), 'len2', len(x2), len(x3)
                    
                    #print 'roi', roi
                    #print 'distance', self.depth_image[roi.y_offset+roi.height/2, roi.x_offset+roi.width/2, 0]
                    cv2.rectangle(cv_image, (roi.x_offset, roi.y_offset), (roi.x_offset+roi.width, roi.y_offset+roi.height), (0, 255, 0), 2)
                    #cv2.rectangle(cv_image, (roi.y_offset+10, roi.x_offset+10), (roi.y_offset+roi.height-10, roi.x_offset+roi.width-10), (0, 0, 255), 2)            
                    cv2.putText(cv_image, 'd:'+'{:.2f}'.format(float(dist)/1000), (roi.x_offset, roi.y_offset), font ,0.5, (255, 0, 0), 2)
				print"distance after tracking",msg_tracking.location
				self.where_pub_tracking.publish(msg_tracking)

            
#tracking detected human by kanmal
	def on_measures(self, measure):
		self.f_gmphd.run_iteration(measure)#, self.born_components)                    
#	def tracking_callback(self, data):
#		if 1:#not self.isBusy:
#			#cv_image = self.rgb_image
#			cv_image = cv2.imread("/home/simonpic/repository/database/GUI/new_gui.png")
#			font = cv2.FONT_HERSHEY_SIMPLEX
#			msg = data
#			msg = where_msgs()
#			msg.header=data.header
#			msg.total=data.total
#			msg.cam_id=data.cam_id
#			msg.roi=data.roi
#			msg.location=data.location
#			msg.user_id=data.user_id
##			msg.location = []
##			msg.user_id = []
#			pscale = 160
#			aa = np.zeros((msg.total, 2))
#			for ii, loc in enumerate(msg.location):
#				if not np.isnan(loc.x):
#					aa[ii] = [loc.x, loc.z]
#					cv2.circle(cv_image, ((int)(loc.x*pscale), (int)(loc.z*pscale-pscale)), 3, (255, 0, 0), 2) 
#				print 'Person', loc.x,loc.z
#			aa = np.transpose(aa)
#			bb=np.zeros((4,aa.shape[1]))
#			bb[0]=aa[0]
#			bb[1]=aa[1]
#			self.f_gmphd.run_iteration(bb)
#			if self.verbose:
#				gms = self.f_gmphd.gm
#				index=0
#				for gm in gms:
#					if gm.detect:# > 0.1:
#						cv2.circle(cv_image, (int(gm.mean[0,0]*pscale), int(gm.mean[1,0]*pscale-pscale)), 3, (0, 255, 0), 2)
#						#cv2.rectangle(cv_image, (x, y), (x2, y2), (0, 255, 0), 2)
##						print"distance after tracking",(gm.mean[0,0]), (gm.mean[1,0])
#						msg.lcation[index].x=(gm.mean[0,0])
#						msg.lcation[index].z=(gm.mean[1,0])
#						print"distance after tracking",msg.lcation[index].x, msg.lcation[index].z
#						index=index+1
#						cv2.putText(cv_image, str(gm.id) +'-{:.2f}'.format(gm.weight), (int(gm.mean[0,0]*pscale), int(gm.mean[1,0]*pscale-pscale)), font ,0.5, (0, 0, 255), 2)
#				cv2.imshow('Tracking: '+self.sensor_name, cv_image)
#				cv2.waitKey(3)
#			return msg
#			self.isBusy = True            
#            #self.where_pub.publish(msg) 


      
def main(args):

  ic = HumanDetectorRGBD()    

  #rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
