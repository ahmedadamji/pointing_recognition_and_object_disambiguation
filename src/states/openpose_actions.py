#!/usr/bin/python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from smach import State

import cv2
import sys
sys.path.append('/tiago_ws/src/openpose/build/python')

from openpose import pyopenpose as op
#from math import atan2, pi
#import time
#import numpy as np

# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

class GetPose(State):
    def __init__(self):
        State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        # start openpose
        params = {}
        params['model_folder'] = '/tiago_ws/src/openpose/models/'
        params['hand'] = True
        params['net_resolution'] = '320x176'
        params["face"] = False
        params["body"] = 1
        self.bridge = CvBridge()
        img_msg = rospy.wait_for_message('/xtion/rgb/image_raw',Image)
        img_msg = rospy.wait_for_message('/xtion/rgb/image_raw',Image)

        # time.sleep(1)
        # print 'TAKING THE PICTURE IN:'
        # for x in range(3,-1,-1):
        #     print x
        #     time.sleep(1)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            
            # grabbed, frame = cap.read()
            # frame = cv2.imread('juan_wave.jpg')
            # do this twice because gazebo sim

            # Starting OpenPose
            opWrapper = op.WrapperPython()
            opWrapper.configure(params)
            opWrapper.start()

            # Process Image
            datum = op.Datum()
            datum.cvInputData = cv_image
            opWrapper.emplaceAndPop([datum])

            human_count = len(datum.poseKeypoints)
            print('Number of humans in frame: {}'.format(human_count))
            print("Body keypoints: \n" + str(datum.poseKeypoints))

            # Display Image
            cv2.imshow("Image Window", datum.cvOutputData)
            cv2.waitKey(0)

        
        except Exception as e:
            print e

        opWrapper.stop()
        return outcome1
