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
import numpy as np

# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

class GetPoseBeforeGazebo:

    # Document somehow that I got this from body_from_camera.py that Juan sent me
    def angle_between_points( a, b, c ):
        ba = np.array(a) - np.array(b)
        bc = np.array(c) - np.array(b)

        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle = np.arccos(cosine_angle)

        # Must check when the distance between two points is 0. In that case return -1.0
        print np.degrees(angle)
        return np.degrees(angle)

    def get_elbow_angle(shoulder,elbow,hand_tip):
        angle = 0
        angle = angle_between_points(shoulder, elbow, hand_tip)
        rospy.loginfo('angle:%f'%(angle))
        return angle

    def get_hand_tip_delta(hand_tip, chest):
        return handTip - chest

    def get_body_points(human, pos):
        pnts = []
        # Link to openpose output data format: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md

        if pos == 'Neck':
            pnt_index = 1
        elif pos == 'RShoulder':
            pnt_index = 2
        elif pos == 'RElbow':
            pnt_index = 3
        elif pos == 'RWrist':
            pnt_index = 4
        elif pos == 'LShoulder':
            pnt_index = 5
        elif pos == 'LElbow':
            pnt_index = 6
        elif pos == 'LWrist':
            pnt_index = 7
        elif pos == 'MidHip':
            pnt_index = 8
        elif pos == 'RHip':
            pnt_index = 9
        elif pos == 'LHip':
            pnt_index = 12
        else:
            rospy.logerr('Unknown  [%s]', pos)
            return None

        pnt = [int(human[pnt_index][0]), int(human[pnt_index][1])]
        return pnt

    def get_hand_points(hand, pos):
        # Link to openpose output data format: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md

        if pos == 'first_finger_tip':
            pnt_index = 8
        elif pos == 'first_finger_dip_joint':
            pnt_index = 7
        elif pos == 'first_finger_pip_joint':
            pnt_index = 6
        elif pos == 'first_finger_mcp_joint':
            pnt_index = 5
        else:
            rospy.logerr('Unknown  [%s]', pos)
            return None
        
        pnt = [int(hand[pnt_index][0]), int(hand[pnt_index][1])]
        return pnt

    def set_params():
        params = dict()
        params['model_folder'] = '/tiago_ws/src/openpose/models/'
        params['hand'] = True
        params['net_resolution'] = '160x80'
        params["face"] = False
        params["body"] = 1
        return params

    def print_body_parameters(datum):
        print("Body keypoints: \n" + str(datum.poseKeypoints))
        print("Face keypoints: \n" + str(datum.faceKeypoints))
        print("Left hand keypoints: \n" + str(datum.handKeypoints[0]))
        print("Right hand keypoints: \n" + str(datum.handKeypoints[1]))


    def execute(self):

        # Set Params
        params = set_params()

        self.bridge = CvBridge()
        img_msg = rospy.wait_for_message('/xtion/rgb/image_raw',Image)
        #img_msg = rospy.wait_for_message('/xtion/rgb/image_raw',Image)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            # Starting OpenPose
            #opWrapper = op.WrapperPython(op.ThreadManagerMode.Synchronous)
            # ^^ This makes the openpose segmentation visible via webcam.
            opWrapper = op.WrapperPython()
            opWrapper.configure(params)
            opWrapper.start()

            # Process Image
            datum = op.Datum()
            datum.cvInputData = cv_image
            opWrapper.emplaceAndPop(op.VectorDatum([datum]))


            # Display Image And Print Body Keypoints
            human_count = len(datum.poseKeypoints)
            print('Number of humans in frame: {}'.format(human_count))
            print_body_parameters(datum)

            for i in range(human_count):
                print('=================================')
                #get_body_angle(datum.poseKeypoints[i], 'waist')
                #angle = get_body_angle(datum.poseKeypoints[i], 'left_elbow')

                chest = get_body_points(datum.poseKeypoints[i], 'Neck')
                right_shoulder = get_body_points(datum.poseKeypoints[i], 'RShoulder')
                left_shoulder = get_body_points(datum.poseKeypoints[i], 'LShoulder')
                right_shoulder = get_body_points(datum.poseKeypoints[i], 'RShoulder')
                left_elbow = get_body_points(datum.poseKeypoints[i], 'LElbow')
                right_elbow = get_body_points(datum.poseKeypoints[i], 'RElbow')
                left_hand_tip = get_hand_points(datum.handKeypoints[0][i], 'first_finger_tip')
                right_hand_tip = get_hand_points(datum.handKeypoints[1][i], 'first_finger_tip')
                left_elbow_angle = get_elbow_angle(left_shoulder,left_elbow,left_hand_tip)
                right_elbow_angle = get_elbow_angle(right_shoulder,right_elbow,right_hand_tip)
                if left_elbow_angle > 120:
                    print('left hand raised')
                elif right_elbow_angle > 120:
                    print('right hand raised')
                else:
                    print('hand not raised')

            cv2.imshow("Image Window", datum.cvOutputData)
            cv2.waitKey(0)
        
        except Exception as e:
            print(e)
            sys.exit(-1)
