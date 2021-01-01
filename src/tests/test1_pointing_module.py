#!/usr/bin/python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import cv2
import sys
sys.path.append('/tiago_ws/src/openpose/build/python')

from openpose import pyopenpose as op
from math import atan2, pi
import time
import numpy as np
import argparse

# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

def get_segmentation():

    try:
    
        # Flags
        parser = argparse.ArgumentParser()
        parser.add_argument("--image_path", default="/tiago_ws/src/pointing_recognition/src/tests/media/image1.png", help="Process an image. Read all standard formats (jpg, png, bmp, etc.).")
        args = parser.parse_known_args()

        # Custom Params
        params = dict()
        params['model_folder'] = '/tiago_ws/src/openpose/models/'
        params['hand'] = True
        params['net_resolution'] = '320x176'
        params["face"] = False
        params["body"] = 1

        # Add others in path?
        for i in range(0, len(args[1])):
            curr_item = args[1][i]
            if i != len(args[1])-1: next_item = args[1][i+1]
            else: next_item = "1"
            if "--" in curr_item and "--" in next_item:
                key = curr_item.replace('-','')
                if key not in params:  params[key] = "1"
            elif "--" in curr_item and "--" not in next_item:
                key = curr_item.replace('-','')
                if key not in params: params[key] = next_item

        # Starting OpenPose
        opWrapper = op.WrapperPython(op.ThreadManagerMode.Synchronous)
        # ^^ This makes the openpose segmentation visible via webcam.
        opWrapper.configure(params)
        opWrapper.execute()


        # try:
        #     grabbed, frame = cap.read()
        #     frame = cv2.imread('juan_wave.jpg')
        #     do this twice because gazebo sim
        #     img_msg = rospy.wait_for_message('/xtion/rgb/image_raw',Image)
        #     img_msg = rospy.wait_for_message('/xtion/rgb/image_raw',Image)
        #     frame = CvBridge().imgmsg_to_cv2(img_msg, "bgr8")

        #     datum = op.Datum()
        #     datum.cvInputData = frame
        #     opWrapper.emplaceAndPop([datum])
        #     image = datum.cvOutputData

        
        # except Exception as e:
        #     print e
        
        # show image
        # cv2.imshow('robocup', image)
        # cv2.waitKey(1)

        # opWrapper.stop()

    except Exception as e:
        print(e)
        sys.exit(-1)

def main(args):

    rospy.init_node('hi')
    get_segmentation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('hi')
    main(sys.argv)