#!/usr/bin/env python
import rospy
import cv2
import math
import numpy as np
import ros_numpy
#import argparse

from cv_bridge import CvBridge, CvBridgeError

from pointing_recognition.srv import PoseKeypoints, PoseKeypointsResponse


from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from sensor_msgs.srv import SetCameraInfo
import sensor_msgs.point_cloud2 as pc2

import sys
sys.path.append("/tiago_ws/src/openpose/build/python")
from openpose import pyopenpose as op


class openpose_server():

    def __init__(self):
        rospy.init_node("openpose_detection")

        self.bridge = CvBridge()
                
        params = self.set_params()
        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(params)
        self.opWrapper.start()


        serv = rospy.Service("openpose_detection", PoseKeypoints, self.compute_pointing)
        rospy.loginfo("openpose_detection service initialised")
        rospy.spin()


    def print_body_parameters(self, datum):
        print("Printing Body Parameters")
        body_keypoints = str(datum.poseKeypoints.astype(float))
        print("Body keypoints: \n" + body_keypoints)
        #print("Face keypoints: \n" + str(np.around(datum.faceKeypoints).fillna(0.0).astype(int)))
        left_hand_keypoints = str(datum.handKeypoints[0].astype(float))
        print("Left hand keypoints: \n" + left_hand_keypoints)
        right_hand_keypoints = str(datum.handKeypoints[1].astype(float))
        print("Right hand keypoints: \n" + right_hand_keypoints)

    # def set_flags(self):
    #     parser = argparse.ArgumentParser()
    #     parser.add_argument("--num_gpu", default=op.get_gpu_number(), help="Number of GPUs.")
    #     return parser.parse_known_args()

    def set_params(self):
        print("Setting OpenPose default parameters")
        params = dict()
        params["body"] = 1
        params["number_people_max"] = 1
        params["model_folder"] = "/tiago_ws/src/openpose/models/"
        params["model_pose"] = "COCO"
        # Even tough 320x320 is dangerously accurate, it is too slow and therefore I
        # will use the fairly accurate 320x240
        params["net_resolution"] = "320x240" # 368x368 (multiples of 16)
        # params["face_net_resolution"] = "160x80" # 368x368 (multiples of 16)
        # params["hand_net_resolution"] = "160x80" # 368x368 (multiples of 16)
        # params["flir_camera"] = True # Used when using Flir camera
        # params["frame_undistort"] = True # Used when simultaneously using FLIR cameras and the 3-D reconstruction module so their camera parameters are read.
        params["hand"] = True
        params["face"] = False
        # params["3d"] = True
        # params["3d_views"] = 2
        return params

    def get_mid_hip_3d(self, left_hip, right_hip):
        x = (left_hip[0]+right_hip[0])/2
        y = (left_hip[1]+right_hip[1])/2
        z = (left_hip[2]+right_hip[2])/2
        mid_hip = [x,y,z]
        return mid_hip
    
    def get_spine_chest_3d(self, neck, mid_hip):
        #The spine_chest refered by microsoft psi for calculating the hand tip delta, with respect to https://docs.microsoft.com/en-us/azure/kinect-dk/body-joints
        # is 2/3 above the mid_hip, between the neck and the mid_hip, therefore as this is not automatically detected, this rough figure will be used, aditionally as this keypoint is not an actual body part
        # There are no papers that give the rative value of distance between these two points (verify this again)
        x = mid_hip[0] + (neck[0]-mid_hip[0])*2/3
        y = mid_hip[1] + (neck[1]-mid_hip[1])*2/3
        z = mid_hip[2] + (neck[2]-mid_hip[2])*2/3
        spine_chest = [x,y,z]
        return spine_chest



    def get_body_points_3d(self, human, pos, xyz_array):
        #print("Requesting body keypoints")
        # Link to openpose output data format: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/02_output.md
        if pos == "Head":
            pnt_index = 0
        elif pos == "Neck":
            pnt_index = 1
        elif pos == "RShoulder":
            pnt_index = 2
        elif pos == "RElbow":
            pnt_index = 3
        elif pos == "RWrist":
            pnt_index = 4
        elif pos == "LShoulder":
            pnt_index = 5
        elif pos == "LElbow":
            pnt_index = 6
        elif pos == "LWrist":
            pnt_index = 7
        elif pos == "RHip":
            pnt_index = 8
        elif pos == "LHip":
            pnt_index = 11
        else:
            rospy.logerr("Unknown  [%s]", pos)
            return None
        
        # the self.get_keypoint function ensures that keypoints below confidence score of a certain value are returned as (nan,nan,nan) to eliminate erronous calculations
        keypoint = self.get_keypoint(human[pnt_index],0.5)
        if (math.isnan(keypoint[0]) or math.isnan(keypoint[0])):
            pnt = [float("NaN"),float("NaN")]
            return self.get_depth(pnt, xyz_array)

        pnt = [int(keypoint[0]), int(keypoint[1])]
        # returns the x,y,z coordinates in meters
        return self.get_depth(pnt, xyz_array)

    def get_hand_points_3d(self, hand, pos, xyz_array):
        #print("Requesting hand keypoints")
        # Link to openpose output data format: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/02_output.md

        if pos == "first_finger_tip":
            pnt_index = 8
        elif pos == "first_finger_dip_joint":
            pnt_index = 7
        elif pos == "first_finger_pip_joint":
            pnt_index = 6
        elif pos == "first_finger_mcp_joint":
            pnt_index = 5
        else:
            rospy.logerr("Unknown  [%s]", pos)
            return None

        # the self.get_keypoint function ensures that keypoints below confidence score of a certain value are returned as (nan,nan,nan) to eliminate erronous calculations
        keypoint = self.get_keypoint(hand[pnt_index],0.2)
        if (math.isnan(keypoint[0]) or math.isnan(keypoint[0])):
            pnt = [float("NaN"),float("NaN")]
            return self.get_depth(pnt, xyz_array)

        pnt = [int(keypoint[0]), int(keypoint[1])]
        # returns the x,y,z coordinates in meters
        return self.get_depth(pnt, xyz_array)

    def get_depth(self, pnt, xyz_array):
        #print("Requesting depth data at requested pixel")
        # Gets the z distance from tiago to the object at pixel x,y in the camera image.
        # x and y are in pixels

        x = pnt[0]
        y = pnt[1]
        if (math.isnan(x) or math.isnan(y)):
            return [float("NaN"), float("NaN"), float("NaN")]


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

    def get_keypoint(self, keypoint, confidence):
        if keypoint[2] > confidence:
            return keypoint
        else:
            # This number is used so that the keypoint lies in the lower left corner of the image which makes the keypoint meaningless
            return [float("NaN"), float("NaN"), float("NaN")]

    def compute_pointing(self, req):

        img_msg = req.img_msg
        depth_points = req.depth_points

        # # Flags
        # args = self.set_flags()

        # # Set Params
        # params = self.set_params()

        # img_msg = rospy.wait_for_message("/xtion/rgb/image_raw",Image)
        
        # To save the depth coordinates once so that I don"t have to call it again and doesnt slow everything
        ## MAKE XYZ_ARRAY A GLOBAL VARIABLE
        # depth_points = rospy.wait_for_message("/xtion/depth_registered/points",PointCloud2)
        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(depth_points, remove_nans=False)
        xyz_array = np.transpose(xyz_array, (1, 0, 2))



        #img_msg2 = rospy.wait_for_message("/xtion/depth_registered/image_raw",Image)
        #print(img_msg.height, img_msg.width)
        #print(img_msg2.height, img_msg2.width)

        try:
            cv2.startWindowThread()
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
            print("Number of humans in frame: {}".format(human_count))
            self.print_body_parameters(datum)
            open_pose_output_image  = datum.cvOutputData
            cv2.imshow("OpenPose Results", open_pose_output_image)
            cv2.waitKey(5000)

            for i in range(human_count):
                print("=================================")
                #get_body_angle(datum.poseKeypoints[i], "waist")
                #angle = get_body_angle(datum.poseKeypoints[i], "left_elbow")

                poseKeypoints = datum.poseKeypoints[i]
                handKeypointsL = datum.handKeypoints[0][i]
                handKeypointsR = datum.handKeypoints[1][i]


                head = self.get_body_points_3d(poseKeypoints, "Head", xyz_array)
                neck = self.get_body_points_3d(poseKeypoints, "Neck", xyz_array)
                right_shoulder = self.get_body_points_3d(poseKeypoints, "RShoulder", xyz_array)
                left_shoulder = self.get_body_points_3d(poseKeypoints, "LShoulder", xyz_array)
                left_elbow = self.get_body_points_3d(poseKeypoints, "LElbow", xyz_array)
                right_elbow = self.get_body_points_3d(poseKeypoints, "RElbow", xyz_array)
                left_wrist = self.get_body_points_3d(poseKeypoints, "LWrist", xyz_array)
                right_wrist = self.get_body_points_3d(poseKeypoints, "RWrist", xyz_array)
                left_hip = self.get_body_points_3d(poseKeypoints, "LHip", xyz_array)
                right_hip = self.get_body_points_3d(poseKeypoints, "RHip", xyz_array)
                mid_hip = self.get_mid_hip_3d(left_hip, right_hip)
                spine_chest = self.get_spine_chest_3d(neck, mid_hip)
                left_hand_tip = self.get_hand_points_3d(handKeypointsL, "first_finger_tip", xyz_array)
                right_hand_tip = self.get_hand_points_3d(handKeypointsR, "first_finger_tip", xyz_array)

                open_pose_output_image_msg = self.bridge.cv2_to_imgmsg(open_pose_output_image, encoding="bgr8")

                # To destroy the cv2 window at the end
                # cv2.destroyAllWindows()

                return PoseKeypointsResponse(head, right_shoulder, left_shoulder, left_elbow, right_elbow, spine_chest, left_hand_tip, right_hand_tip, left_wrist, right_wrist, open_pose_output_image_msg)


               

        
        except Exception as e:
            print(e)
            sys.exit(-1)


if __name__ == "__main__":
    openpose_server()
