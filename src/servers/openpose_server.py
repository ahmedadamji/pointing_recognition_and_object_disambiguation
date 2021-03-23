#!/usr/bin/env python
import rospy
import cv2
#import math
import numpy as np
import ros_numpy
#import argparse

from cv_bridge import CvBridge, CvBridgeError

from pointing_recognition.srv import OpenPoseKeypoints, OpenPoseKeypointsResponse


from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from sensor_msgs.srv import SetCameraInfo
import sensor_msgs.point_cloud2 as pc2

import sys
sys.path.append('/tiago_ws/src/openpose/build/python')
from openpose import pyopenpose as op


class openpose_server():

    def __init__(self):
        rospy.init_node('openpose_detection')

        self.bridge = CvBridge()
                
        params = self.set_params()
        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(params)
        self.opWrapper.start()


        serv = rospy.Service('openpose_detection', OpenPoseKeypoints, self.compute_pointing)


        rospy.loginfo('openpose_detection service initialised')
        rospy.spin()

    
    def angle_between_points( self, a, b, c ):
        rospy.loginfo('Calculating angle between points')
        ba = np.array(a) - np.array(b)
        bc = np.array(c) - np.array(b)

        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle = np.arccos(cosine_angle)

        print np.degrees(angle)
        return np.degrees(angle)

    def get_elbow_angle(self, shoulder,elbow,hand_tip, hand):
        rospy.loginfo('Calculating elbow angle')
        angle = 0
        angle = self.angle_between_points(shoulder, elbow, hand_tip)
        rospy.loginfo('%s angle:%f'%(hand,angle))
        return angle

    def get_hand_tip_delta(self, hand_tip, chest, hand):
        # After testing, change hand tip delta as a test parameter for is pointing or not to height difference between hand and chest
        rospy.loginfo('Calculating hand tip delta')
        #comparing the x coordinate of chest and hand tip
        handtipdelta = hand_tip[0] - chest[0]
        rospy.loginfo('%s handtipdelta:%f'%(hand, handtipdelta))
        return handtipdelta


    def print_body_parameters(self, datum):
        rospy.loginfo('Printing Body Parameters')
        print("Body keypoints: \n" + str(np.around(datum.poseKeypoints).astype(int)))
        #print("Face keypoints: \n" + str(np.around(datum.faceKeypoints).fillna(0.0).astype(int)))
        print("Left hand keypoints: \n" + str(np.around(datum.handKeypoints[0]).astype(int)))
        print("Right hand keypoints: \n" + str(np.around(datum.handKeypoints[1]).astype(int)))

    # def set_flags(self):
    #     parser = argparse.ArgumentParser()
    #     parser.add_argument("--num_gpu", default=op.get_gpu_number(), help="Number of GPUs.")
    #     return parser.parse_known_args()

    def set_params(self):
        rospy.loginfo('Setting OpenPose default parameters')
        params = dict()
        params["body"] = 1
        params["number_people_max"] = 1
        params['model_folder'] = '/tiago_ws/src/openpose/models/'
        params['model_pose'] = 'COCO'
        # Even tough 320x320 is dangerously accurate, it is too slow and therefore I
        # will use the fairly accurate 320x240
        params['net_resolution'] = '320x240' # 368x368 (multiples of 16)
        # params['face_net_resolution'] = '160x80' # 368x368 (multiples of 16)
        # params['hand_net_resolution'] = '160x80' # 368x368 (multiples of 16)
        # params['flir_camera'] = True # Used when using Flir camera
        # params['frame_undistort'] = True # Used when simultaneously using FLIR cameras and the 3-D reconstruction module so their camera parameters are read.
        params['hand'] = True
        params['face'] = False
        # params["3d"] = True
        # params['3d_views'] = 2
        return params

    def get_body_points_3d(self, human, pos, xyz_array):
        #rospy.loginfo('Requesting body keypoints')
        # Link to openpose output data format: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md
        if pos == 'Head':
            pnt_index = 0
        elif pos == 'Neck':
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
        # returns the x,y,z coordinates in meters
        return self.get_depth(pnt, xyz_array)

    def get_hand_points_3d(self, hand, pos, xyz_array):
        #rospy.loginfo('Requesting hand keypoints')
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
        # returns the x,y,z coordinates in meters
        return self.get_depth(pnt, xyz_array)

    def get_depth(self, pnt, xyz_array):
        #rospy.loginfo('Requesting depth data at requested pixel')
        # Gets the z distance from tiago to the object at pixel x,y in the camera image.
        # x and y are in pixels
        x = pnt[0]
        y = pnt[1]


        #print(np.shape(xyz_array))
        # transposing to get output using format xyz_array[x][y] instead of xyz_array[y][x]
        #print(xyz_array[487][224]) # reading random coordinate, supposed to be chest / head
        # To check the range of the rgbd camera
        #print(xyz_array[479][639])
        #print(xyz_array[0][0])
        #print(depth_points.header.frame_id)
        #print(depth_points.height, depth_points.width)

        # xyz_array returns in meters
        return xyz_array[x][y]

    def compute_pointing(self, req):

        img_msg = req.img_msg
        depth_points = req.depth_points

        # # Flags
        # args = self.set_flags()

        # # Set Params
        # params = self.set_params()

        # img_msg = rospy.wait_for_message('/xtion/rgb/image_raw',Image)
        
        # To save the depth coordinates once so that I don't have to call it again and doesnt slow everything
        ## MAKE XYZ_ARRAY A GLOBAL VARIABLE
        # depth_points = rospy.wait_for_message('/xtion/depth_registered/points',PointCloud2)
        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(depth_points, remove_nans=False)
        xyz_array = np.transpose(xyz_array, (1, 0, 2))



        #img_msg2 = rospy.wait_for_message('/xtion/depth_registered/image_raw',Image)
        #print(img_msg.height, img_msg.width)
        #print(img_msg2.height, img_msg2.width)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            # # Starting OpenPose
            # #opWrapper = op.WrapperPython(op.ThreadManagerMode.Synchronous)
            # # ^^ This makes the openpose segmentation visible via webcam.
            # self.opWrapper = op.WrapperPython()
            # self.opWrapper.configure(params)
            # self.opWrapper.start()

            # Process Image
            datum = op.Datum()
            datum.cvInputData = cv_image
            self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))


            # Display Image And Print Body Keypoints
            human_count = len(datum.poseKeypoints)
            print('Number of humans in frame: {}'.format(human_count))
            self.print_body_parameters(datum)
            open_pose_output_image  = datum.cvOutputData
            cv2.imshow("OpenPose Results", open_pose_output_image)
            cv2.waitKey(5000)

            for i in range(human_count):
                print('=================================')
                #get_body_angle(datum.poseKeypoints[i], 'waist')
                #angle = get_body_angle(datum.poseKeypoints[i], 'left_elbow')

                poseKeypoints = datum.poseKeypoints[i]
                handKeypointsL = datum.handKeypoints[0][i]
                handKeypointsR = datum.handKeypoints[1][i]


                head = self.get_body_points_3d(poseKeypoints, 'Head', xyz_array)
                chest = self.get_body_points_3d(poseKeypoints, 'Neck', xyz_array)
                right_shoulder = self.get_body_points_3d(poseKeypoints, 'RShoulder', xyz_array)
                left_shoulder = self.get_body_points_3d(poseKeypoints, 'LShoulder', xyz_array)
                right_shoulder = self.get_body_points_3d(poseKeypoints, 'RShoulder', xyz_array)
                left_elbow = self.get_body_points_3d(poseKeypoints, 'LElbow', xyz_array)
                right_elbow = self.get_body_points_3d(poseKeypoints, 'RElbow', xyz_array)
                left_hand_tip = self.get_hand_points_3d(handKeypointsL, 'first_finger_tip', xyz_array)
                right_hand_tip = self.get_hand_points_3d(handKeypointsR, 'first_finger_tip', xyz_array)
                left_elbow_angle = self.get_elbow_angle(left_shoulder,left_elbow,left_hand_tip,'left')
                right_elbow_angle = self.get_elbow_angle(right_shoulder,right_elbow,right_hand_tip,'right')
                left_hand_tip_delta = self.get_hand_tip_delta(left_hand_tip,chest,'left')
                right_hand_tip_delta = self.get_hand_tip_delta(right_hand_tip,chest,'right')


                open_pose_output_image_msg = self.bridge.cv2_to_imgmsg(open_pose_output_image, encoding="bgr8")

                # Parameters that need to be satisfied in case hand is pointing based on observed data points
                # Later try to shift this functionality to is_pointing() funtion

                if ((left_elbow_angle > 120)and(abs(left_hand_tip_delta)>0.5)):
                    print('left hand pointing')
                    hand = 'left'

                    return OpenPoseKeypointsResponse(hand, left_hand_tip, head, open_pose_output_image_msg)

                    
                elif ((right_elbow_angle > 120)and(abs(right_hand_tip_delta)>0.5)):
                    print('right hand pointing')
                    hand = 'right'

                    return OpenPoseKeypointsResponse(hand, right_hand_tip, head, open_pose_output_image_msg)


                else:
                    print('hand not pointing')
                    hand = 'none_pointing'

                

        
        except Exception as e:
            print(e)
            sys.exit(-1)


if __name__ == '__main__':
    openpose_server()
