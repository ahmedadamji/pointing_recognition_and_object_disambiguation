#!/usr/bin/python

import cv2
import numpy as np
import sys
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from openpose import pyopenpose as op
from tempfile import TemporaryFile



def angle_between_points( p0, p1, p2 ):
    a = np.array([p0[0],p0[1]])
    b = np.array([p1[0],p1[1]])
    c = np.array([p2[0],p2[1]])

    ba = a - b
    bc = c - b

    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)

    # Must check when the distance between two points is 0. In that case return -1.0
    print np.degrees(angle)
    return np.degrees(angle)

        
def get_angle_point(human, pos):
    pnts = []

    if pos == 'left_elbow':
        pos_list = (5,6,7)
    elif pos == 'left_hand':
        pos_list = (5,6,7)
    elif pos == 'left_knee':
        pos_list = (12,13,14)
    elif pos == 'left_ankle':
        pos_list = (5,12,14)
    elif pos == 'right_elbow':
        pos_list = (2,3,4)
    elif pos == 'right_hand':
        pos_list = (1,2,4)
    elif pos == 'right_knee':
        pos_list = (9,10,11)
    elif pos == 'right_ankle':
        pos_list = (2,9,11)
    elif pos == 'waist':
        pos_list = (1,8,10)
    else:
        rospy.logerr('Unknown  [%s]', pos)
        return pnts

    for i in range(3):
        if human[pos_list[i]][2] <= 0.1:
            print('component [%d] incomplete'%(pos_list[i]))
            return pnts

        pnts.append((int( human[pos_list[i]][0]), int( human[pos_list[i]][1])))
    return pnts


def get_angle(human, part):
    pnts = get_angle_point(human, part)
    if len(pnts) != 3:
        rospy.logerr('component incomplete')
        return

    angle = 0
    if pnts is not None:
        angle = angle_between_points(pnts[0], pnts[1], pnts[2])
        rospy.loginfo('%s angle:%f'%(part, angle))
    return angle

class image_converter:
    def __init__(self):
        # self.image_pub = rospy.Publisher("/ros_to_cv2", Image, queue_size=3)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.callback)

    

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        params = dict()
        params["model_folder"] = "/home/juancm/openpose/models/"
        # params["face"] = False
        # params["body"] = 1
        # params["hand"] = True
        # params["hand_detector"] = 2

        # Starting OpenPose
        opWrapper = op.WrapperPython()
        opWrapper.configure(params)
        opWrapper.start()

        # Process Image
        datum = op.Datum()
        datum.cvInputData = cv_image

        # print '##########################################################'
        # print(datum.poseScores)
        # poseModel = op.PoseModel.BODY_25
        # print(op.getPoseBodyPartMapping(poseModel))
        # print(op.getPoseNumberBodyParts(poseModel))
        # print(op.getPosePartPairs(poseModel))
        # print(op.getPoseMapIndex(poseModel))
        # print '##########################################################'

        # print '##########################################################'
        # print(datum.poseKeypoints3D)
        # print '##########################################################'

        # # np.savetxt('data1.csv', datum.handKeypoints[0][0], delimiter=',', fmt='%d')

        # print("Body keypoints: \n" + str(datum.poseKeypoints))
        # print("Face keypoints: \n" + str(datum.faceKeypoints))
        # print '##########################################################'
        # print(datum.poseScores)
        # print '##########################################################'
        # print("Left hand keypoints: \n" + str(datum.handKeypoints[0]))
        # print("Right hand keypoints: \n" + str(datum.handKeypoints[1]))
        # print '##########################################################'
        # print(datum.poseScores)
        # print '##########################################################'
        

        opWrapper.emplaceAndPop([datum])

        human_count = len(datum.poseKeypoints)
        print('Number of humans in frame: {}'.format(human_count))
        print("Body keypoints: \n" + str(datum.poseKeypoints))
        for i in range(human_count):
            print('=================================')
            get_angle(datum.poseKeypoints[i], 'waist')
            angle = get_angle(datum.poseKeypoints[i], 'left_hand')
            if angle > 154 and angle < 163:
                # font 
                font = cv2.FONT_HERSHEY_SIMPLEX 
                
                # org 
                org = (50, 50) 
                
                # fontScale 
                fontScale = 1
                
                # Blue color in BGR 
                color = (255, 0, 0) 
                
                # Line thickness of 2 px 
                thickness = 2
                datum.cvOutputData = cv2.putText(datum.cvOutputData, 'OpenCV', org, font, fontScale, color, thickness, cv2.LINE_AA) 
        
        cv2.imshow("Image Window", datum.cvOutputData)
        cv2.waitKey(0)

        


    
def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    





