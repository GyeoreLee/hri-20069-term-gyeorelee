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
import gmphdROI as gmphdroi
import pickle
where_mutex = Lock()

fusion_mutex = Lock()

class HumanDetectorRGBD:
    def __init__(self):
#    	kalman tracking parameter setting for tracking ROI
		sigma_q_roi = 67
		sigma_r_roi = 60
		sigma_p0_roi = 55
		sigma_v_roi = 2
		p_d_roi = 0.25
		p_s_roi = 0.59
		merge_thresh_roi = 30
		F_roi = [[1, 0, 0, 0, 1, 0, 0, 0], 
                [0, 1, 0, 0, 0, 1, 0 ,0], 
                [0, 0, 1, 0, 0, 0 ,1, 0], 
                [0, 0, 0, 1, 0, 0, 0, 1],
                [0, 0, 0, 0, 1, 0, 0 ,0],                 
                [0, 0, 0, 0, 0, 1, 0 ,0], 
                [0, 0, 0, 0, 0, 0 ,1, 0], 
                [0, 0, 0, 0, 0, 0, 0, 1]]
		v_roi = math.pow(sigma_v_roi, 2)
		q_roi = math.pow(sigma_q_roi, 2)
		Q_roi = [[q_roi, 0, 0, 0, 0, 0, 0, 0], 
                [0, q_roi, 0, 0, 0, 0, 0 ,0], 
                [0, 0, q_roi, 0, 0, 0, 0, 0], 
                [0, 0, 0, q_roi, 0, 0, 0, 0],
                [0, 0, 0, 0, v_roi, 0, 0, 0],                 
                [0, 0, 0, 0, 0, v_roi, 0 ,0], 
                [0, 0, 0, 0, 0, 0 ,v_roi, 0], 
                [0, 0, 0, 0, 0, 0, 0, v_roi]]
		H_roi = [[1, 0, 0, 0, 0, 0, 0, 0], 
                [0, 1, 0, 0, 0, 0, 0 ,0], 
                [0, 0, 1, 0, 0, 0 ,0, 0], 
                [0, 0, 0, 1, 0, 0, 0, 0]]
		R_roi = [[math.pow(sigma_r_roi, 2), 0, 0, 0], [0, math.pow(sigma_r_roi, 2), 0, 0], [0, 0, math.pow(sigma_r_roi, 2), 0], [0, 0, 0, math.pow(sigma_r_roi, 2)]]
		p_roi = math.pow(sigma_p0_roi, 2)
		P0_roi = [[p_roi, 0, 0, 0, 0, 0, 0, 0], 
                [0, p_roi, 0, 0, 0, 0, 0 ,0], 
                [0, 0, p_roi, 0, 0, 0, 0, 0], 
                [0, 0, 0, p_roi, 0, 0, 0, 0],
                [0, 0, 0, 0, v_roi, 0, 0, 0],                 
                [0, 0, 0, 0, 0, v_roi, 0 ,0], 
                [0, 0, 0, 0, 0, 0 ,v_roi, 0], 
                [0, 0, 0, 0, 0, 0, 0, v_roi]]
		clutter_intensity_roi = 0.0
		self.f_gmphd_roi = gmphdroi.GMPHD([], p_s_roi, p_d_roi, F_roi, Q_roi, H_roi, R_roi, P0_roi, clutter_intensity_roi, merge_thresh_roi)
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
		self.init_variable()
#sub and pub toptics
		rospy.init_node('conver_roi_location_kinect2', anonymous=False)
		self.camera = str(rospy.get_param('~camera', 1))
		self.verbose = rospy.get_param('~verbose', True)
		self.sensor_name = 'sn_kinect_' + self.camera
		self.depth_image = None
		self.counter = 0
		
		self.image_pub = rospy.Publisher("/image_topic_"+self.camera, Image, queue_size=1)
		self.where_pub = rospy.Publisher("/sn_kinect/detector", where_msgs, queue_size = 1)
		self.where_pub_tracking = rospy.Publisher("/sn_kinect/detector_tracking", where_msgs, queue_size = 1)
		self.bridge = CvBridge()
		self.where_sub = rospy.Subscriber("/sn_kinect/detector2", where_msgs, self.where_callback, queue_size = 1)
		self.depth_sub = rospy.Subscriber("/sn_kinect"+self.camera+"/depth/image_raw2", Image, self.depth_callback, queue_size = 1)
		self.compImage_sub = rospy.Subscriber("/sn_kinect"+self.camera+"/rgb/image_color/compressed", CompressedImage, self.compressedImage_callback,  queue_size = 1)
		plt.ion()
		rospy.Timer(rospy.Duration(0.01), self.timer_callback)
#		print 'Start Tracker', self.sensor_name
		self.isBusy = True            
            
    
    def init_variable(self):
#        self.kinect1_timer = rospy.get_time()
        self.kinect1_tmp = where_msgs()
#        self.kinect2_timer = rospy.get_time()
        self.kinect2_tmp = where_msgs()
#        self.kinect3_timer = rospy.get_time()
        self.kinect3_tmp = where_msgs()
#        self.kinect4_timer = rospy.get_time()
        self.kinect4_tmp =where_msgs()
    def rgb_callback(self,data):
        self.isBusy = True    
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
          print e

        (rows,cols,channels) = cv_image.shape
        # Detect all object classes and regress object bounds
        self.rgb_image = cv_image
        self.isBusy = False              

    def depth_callback(self,data):
	self.depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
                                      
                  
    def compressedImage_callback(self,data):
        self.isBusy = True    
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(data.data, np.uint8)
        #print 'shape rgb', np_arr.shape        
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        
        self.rgb_image = image_np
        self.isBusy = False           


    def where_callback(self, data):
        #print '===data', data
        if len(data.cam_id):
	        #print 'camid', data.cam_id[n].data
	        #convert to global location by matching 'cam_id' and '/psn_unit' values            
	        if data.cam_id[0].data == 'sn_kinect_1':
	            self.kinect1_tmp = data
	            self.kinect1_timer = rospy.get_time()
	            #print '--data', data
	            #print '-----k1', self.kinect1_tmp

	        if data.cam_id[0].data == 'sn_kinect_2':
	            self.kinect2_tmp = data
	            self.kinect2_timer = rospy.get_time()

	        if data.cam_id[0].data == 'sn_kinect_3':
	            self.kinect3_tmp = data
	            self.kinect3_timer = rospy.get_time()

	        if data.cam_id[0].data == 'sn_kinect_4':
	            self.kinect4_tmp = data
	            self.kinect4_timer = rospy.get_time()

                            

    def timer_callback(self, event):
    	fusion_mutex.acquire()
    	data=self.kinect2_tmp
    	if not self.counter:#self.depth_image: 
            self.prev_msg = data        
        self.counter += 1
        if not self.isBusy:
            cv_image_rgb = self.rgb_image            
            font = cv2.FONT_HERSHEY_SIMPLEX  
#            (rows,cols,channels) = self.depth_image.shape
            rows =480
            cols=640              
            #publish data in where_msgs
            msg_detect = where_msgs()
	    if data.total>0:
	    	if data.cam_id[0].data=='sn_kinect_2':
#		    print"testtttttttttttttttttttt"
		    msg_detect.header=data.header
		    msg_detect.total=data.total
		    msg_detect.cam_id=data.cam_id
		    msg_detect.roi=data.roi
		    msg_detect.location = []
		    msg_detect.user_id = []
            index=0
            
            if len(msg_detect.roi)>0:
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
#            print "republish message detection",msg_detect.location
            if len(msg_detect.roi):
				for roi in msg_detect.roi:
					if roi.width*roi.height>1000:
						cv2.rectangle(cv_image_rgb, (roi.x_offset, roi.y_offset), (roi.x_offset+roi.width, roi.y_offset+roi.height), (0, 255, 0), 2)
            self.prev_msg = msg_detect

            self.isBusy = True            
            self.where_pub.publish(msg_detect)         
            #self.depth_image = self.new_depth
            
            
            if 1:#not self.isBusy:
				#cv_image = self.rgb_image
								
				cv_image = cv2.imread("/home/psn2/repository//psn_unit/sp_hl_hd_op/convertwhere/floor.jpg")
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
						
						cv2.circle(cv_image, ((int)(loc.x*pscale)+150, (int)(loc.z*pscale-pscale)), 3, (255, 0, 0), 2) 
#					print 'Person', loc.x,loc.z
				aa = np.transpose(aa)
				bb=np.zeros((4,aa.shape[1]))
				bb[0]=aa[0]
				bb[1]=aa[1]
				self.f_gmphd.run_iteration(bb)
				if self.verbose:
					gms = self.f_gmphd.gm
					index=0

					for gm in gms:
						if gm.detect and len(msg_tracking.location)>0  :# > 0.1:
							cv2.circle(cv_image, (int(gm.mean[0,0]*pscale+150), int(gm.mean[1,0]*pscale-pscale)), 3, (0, 255, 0), 2)
							#cv2.rectangle(cv_image, (x, y), (x2, y2), (0, 255, 0), 2)
							if isinstance(gm.mean[0,0], float) == True and isinstance(gm.mean[1,0], float)== True:
								msg_tracking.location[msg_tracking.total-1-index].x=(gm.mean[0,0])
								msg_tracking.location[msg_tracking.total-1-index].z=(gm.mean[1,0])
							
							index=index+1
#							cv2.putText(cv_image, str(gm.id) +'-{:.2f}'.format(gm.weight), (int(gm.mean[0,0]*pscale)+150, int(gm.mean[1,0]*pscale-pscale)), font ,0.5, (0, 0, 255), 2)
#					
#					cv2.imshow('Tracking: '+self.sensor_name, cv_image)
#					cv2.waitKey(3)
				
				
#				if self.verbose: 
#					cv2.rectangle(cv_image, (roi.x_offset, roi.y_offset), (roi.x_offset+roi.width, roi.y_offset+roi.height), (0, 255, 0), 2)
#					cv2.putText(cv_image, 'd:'+'{:.2f}'.format(float(dist)/1000), (roi.x_offset, roi.y_offset), font ,0.5, (255, 0, 0), 2)
				
##tracking ROI by kalman
#				aa_roi = np.zeros((msg_tracking.total, 8))
#				for ii, roi in enumerate(msg_tracking.roi):
#					aa_roi[ii] = [roi.x_offset, roi.y_offset, roi.x_offset+roi.width, roi.y_offset+roi.height, 0, 0, 0, 0]
#				if not len(aa_roi):
#					aa_roi = np.random.rand(1, 8)*10000

#				if len(aa_roi):
#					bb_roi = np.transpose(aa_roi)
#					tic = time()
#					self.f_gmphd_roi.run_iteration(bb_roi)
#				if self.verbose:
#					gms_roi = self.f_gmphd_roi.gm
#					index=0
#					for gm in gms_roi:
#						if gm.detect:# > 0.1:
#							if isinstance(msg_tracking.location[msg_tracking.total-1-index].x, float)== True :
#								pos = "x:"+"{0:.2f}".format(msg_tracking.location[msg_tracking.total-1-index].x) + ",z:" + "{0:.2f}".format(msg_tracking.location[msg_tracking.total-1-index].z)

#							
##							if isinstance((gm.mean[0]), int)== True:
#							print"((gm.mean[2])-(gm.mean[0]))*((gm.mean[3])-(gm.mean[1]))",((gm.mean[2])-(gm.mean[0]))*((gm.mean[3])-(gm.mean[1]))
#							if ((gm.mean[2])-(gm.mean[0]))*((gm.mean[3])-(gm.mean[1]))>1000:
#								cv2.rectangle(cv_image_rgb, (int(gm.mean[0]), int(gm.mean[1])), (int(gm.mean[2]), int(gm.mean[3])), (0, 0, 255), 2)                        
#								center_name = (int(gm.mean[0]), (int(gm.mean[1])+15))
#								cv2.putText(cv_image_rgb, pos,center_name, font ,0.5, (255, 0, 0), 2)
#							if isinstance((gm.mean[0]), int)== True and isinstance((gm.mean[1]), int) :
#								msg_tracking.roi[msg_tracking.total-1-index].x_offset=(gm.mean[0])
#								msg_tracking.roi[msg_tracking.total-1-index].y_offset=(gm.mean[1])
#								msg_tracking.roi[msg_tracking.total-1-index].width=(gm.mean[2])-(gm.mean[0])
#								msg_tracking.roi[msg_tracking.total-1-index].height=(gm.mean[3])-(gm.mean[1])
#							index=index+1
				small = cv2.resize(cv_image_rgb, (0,0), fx=0.6, fy=0.6)                     
				cv2.imshow('Tracking ROI: '+self.sensor_name, small)
				cv2.waitKey(3)
				self.where_pub_tracking.publish(msg_tracking)

            
        fusion_mutex.release()

      
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
