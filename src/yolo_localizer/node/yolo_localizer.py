#! /usr/bin/env python

import rospy
import numpy as np
import cv2
import sys

from sensor_msgs.msg import Image,CompressedImage
from yolo_localizer_msgs.msg import where_msgs, bbox_array
from munkres import Munkres
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

rgb_image = np.zeros((480, 640, 3), np.uint8)
yolo_human_boxes = []
yolo_human_3Dpos = [] # To be assigned
openNI_human_boxes = []
openNI_human_3Dpos = [] # Parsed from NI msg

pub = 0

def detection_parser(where_msgs):

    if where_msgs.cam_id[0].data == 'sn_kinect_3':
        global openNI_human_boxes, openNI_human_3Dpos
        openNI_human_boxes = []
        openNI_human_3Dpos = []

        for index in range(where_msgs.total):
            xmin = where_msgs.roi[index].x_offset
            xmax = where_msgs.roi[index].x_offset + where_msgs.roi[index].width
            ymin = where_msgs.roi[index].y_offset
            ymax = where_msgs.roi[index].y_offset + where_msgs.roi[index].height

            openNI_human_boxes.append([xmin, ymin, xmax, ymax])

            x = where_msgs.location[index].x
            y = where_msgs.location[index].y
            z = where_msgs.location[index].z
            openNI_human_3Dpos.append([x, y, z])


def yolo_bbox_parser(bbox_array):
    global yolo_human_boxes
    yolo_human_boxes = []

    for bbox in bbox_array.bboxes:
        if bbox.Class == 'person':
            yolo_human_boxes.append([bbox.xmin,bbox.ymin,bbox.xmax,bbox.ymax ])
    #print bbox_array.bboxes[0].Class
    #for box in bbox_array.bboxes:
    #    print box[0]

def imageDisplay(rgb_image):

    global yolo_human_3Dpos,yolo_human_boxes
    global openNI_human_3Dpos, openNI_human_boxes

    if yolo_human_boxes :
        for i, human_box in enumerate(yolo_human_boxes):
            cv2.rectangle(rgb_image, (human_box[0], human_box[1]),
                          (human_box[2], human_box[3]), (255, 207, 0), 2) #sky blue
            # 3D pos display
            if yolo_human_3Dpos and yolo_human_boxes.__len__() == yolo_human_3Dpos.__len__():
                yolo_x = yolo_human_3Dpos[i][0]
                yolo_y = yolo_human_3Dpos[i][1]
                yolo_z = yolo_human_3Dpos[i][2]
                cv2.putText(rgb_image, str('X:%.2f Y:%2.f Z:%.2f' % (yolo_x, yolo_y, yolo_z)),
                            (human_box[0], human_box[1])
                            , cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 207, 0), 2)

    if openNI_human_boxes :
        for i, human_box in enumerate(openNI_human_boxes):
            cv2.rectangle(rgb_image, (human_box[0], human_box[1]),
                          (human_box[2], human_box[3]), (68, 231, 98), 2) # green
            # 3D pos display
            if openNI_human_3Dpos and openNI_human_boxes.__len__() == openNI_human_3Dpos.__len__():

                NI_x = openNI_human_3Dpos[i][0]
                NI_y = openNI_human_3Dpos[i][1]
                NI_z = openNI_human_3Dpos[i][2]
                cv2.putText(rgb_image, str('X:%.2f Y:%2.f Z:%.2f' % (NI_x, NI_y, NI_z)),
                            (human_box[0], human_box[3])
                            , cv2.FONT_HERSHEY_SIMPLEX, 0.8, (68, 231, 98), 2)



    cv2.imshow('Color2', rgb_image)
    cv2.waitKey(1)

def association(self):

    global yolo_human_3Dpos
    global openNI_human_3Dpos
    yolo_human_3Dpos = []

    # condition check
    if yolo_human_boxes and openNI_human_boxes:
        presize = [yolo_human_boxes.__len__(), openNI_human_boxes.__len__()]
        cost_matrix = np.zeros(yolo_human_boxes.__len__()*openNI_human_boxes.__len__())

        matrix_index = 0

        for yolo in yolo_human_boxes:
            for NI in openNI_human_boxes:

                # Area calculation
                if not((yolo[0] > NI[2]) or (yolo[2] < NI[0]) or
                        (yolo[1] > NI[3]) or (yolo[3] < NI[1])): # 0 : xmin, 1: ymin, 2: xmax, 3: ymax

                    rect_x = np.max([yolo[0], NI[0]])
                    rect_y = np.max([yolo[1], NI[1]])
                    rect_width = np.min([yolo[2],NI[2]])- rect_x
                    rect_height = np.min([yolo[3],NI[3]])- rect_y


                    cost_matrix[matrix_index] = costfunction(rect_width,rect_height)


                matrix_index += 1
        try:
            cost_matrix.shape =  (yolo_human_boxes.__len__(),openNI_human_boxes.__len__())
        except:
            print 'pre-size' , presize
            print 'postsize',[yolo_human_boxes.__len__(),openNI_human_boxes.__len__()]

            return 0

        print 'cost_matrix\n',cost_matrix

        # Optimal assignment
        m = Munkres()
        try:
            assign_matrix = m.compute(310000 - cost_matrix)
        except:
            assign_matrix =  m.compute(310000 - cost_matrix.T)


        #print assign_matrix

        # NI 3D pose assignment
        for item in assign_matrix:
            index = item[1]
            #print openNI_human_3Dpos[index]
            try:
                yolo_human_3Dpos.append(openNI_human_3Dpos[index])
            except:
                continue
    empty_msg = where_msgs()
    pub.publish(empty_msg)


def costfunction(horizon, vertical):
    return horizon*vertical


def compressedImage_callback(CompressedImage):

    np_arr = np.fromstring(CompressedImage.data, np.uint8)
    # print 'shape rgb', np_arr.shape
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    global rgb_image
    rgb_image = np.copy(image_np)
    imageDisplay(rgb_image)
    #cv2.imshow('Color', rgb_image)
    #cv2.waitKey(1)




def main():

    rospy.init_node('yolo_localizer', anonymous=True)


    rospy.Subscriber('/sn_kinect3/rgb/image_color/compressed', CompressedImage, compressedImage_callback,
                     queue_size=1, buff_size=52428800)

    rospy.Subscriber("/sn_kinect/detector", where_msgs, detection_parser, queue_size=1, buff_size=52428800 * 4)
    rospy.Subscriber("/YOLO_bboxes", bbox_array, yolo_bbox_parser, queue_size=1, buff_size=52428800 * 4)

    global pub
    pub = rospy.Publisher("/yolo_localize_out", where_msgs, queue_size=1)

    rospy.Timer(rospy.Duration(0.033),association)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
