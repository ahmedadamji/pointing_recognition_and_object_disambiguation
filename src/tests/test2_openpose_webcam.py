#!/usr/bin/python
import rospy
import cv2
import sys
sys.path.append('/tiago_ws/src/openpose/build/python')
from openpose import pyopenpose as op


def get_segmentation_from_webcam():
    try:

        # Custom Params
        params = dict()
        params['model_folder'] = '/tiago_ws/src/openpose/models/'
        params['model_pose'] = 'COCO'
        params['net_resolution'] = '320x176'
        params['hand'] = True
        params["face"] = False
        params["body"] = 1


        # Starting OpenPose
        opWrapper = op.WrapperPython(op.ThreadManagerMode.Synchronous)
        opWrapper.configure(params)
        opWrapper.execute()

    except Exception as e:
        print(e)
        sys.exit(-1)

def main(args):
    get_segmentation_from_webcam()
    rospy.init_node('hi')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)