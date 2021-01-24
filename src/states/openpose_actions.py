#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from smach import State

import sensor_msgs.point_cloud2 as pc2

import cv2
import sys
sys.path.append('/tiago_ws/src/openpose/build/python')

from openpose import pyopenpose as op
#from math import atan2, pi
#import time
import numpy as np
import open3d as o3d
import sympy
from sympy import Point3D
from sympy.abc import L
from sympy.geometry import Line3D, Segment3D
import ros_numpy
import argparse

# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

class GetPose(State):
    def __init__(self):
        State.__init__(self, outcomes=['outcome1', 'outcome2'])

        # Document somehow that I got this from body_from_camera.py that Juan sent me
    def angle_between_points( self, a, b, c ):
        ba = np.array(a) - np.array(b)
        bc = np.array(c) - np.array(b)

        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle = np.arccos(cosine_angle)

        # Must check when the distance between two points is 0. In that case return -1.0
        print np.degrees(angle)
        return np.degrees(angle)

    def get_elbow_angle(self, shoulder,elbow,hand_tip, hand):
        angle = 0
        angle = self.angle_between_points(shoulder, elbow, hand_tip)
        rospy.loginfo('%s angle:%f'%(hand,angle))
        return angle

    def get_hand_tip_delta(self, hand_tip, chest, hand):
        #comparing the x coordinate of chest and hand tip
        handtipdelta = hand_tip[0] - chest[0]
        rospy.loginfo('%s handtipdelta:%f'%(hand, handtipdelta))
        return handtipdelta

<<<<<<< HEAD
    def normalize(self, array):
        normalized = array/np.linalg.norm(array, axis = 0)
        return normalized

    def get_pointing_line(self, hand_tip, head, points):
        # https://github.com/mikedh/trimesh/blob/master/examples/ray.py
        # https://github.com/mikedh/trimesh/issues/211
        # find a way to get the depth mesh from the rgbd camera

        direction = self.normalize(hand_tip - head)
        print(np.shape(direction))
        ray_directions = direction
        # Do not start the line right at the hand-tip to avoid having an intersection with the mesh around the hand.
        start_point = hand_tip + (direction*0.2)
        ray_origins = start_point
        end_point = hand_tip + (direction*0.3)
        line = Line3D(Point3D(np.array(start_point)[0],np.array(start_point)[1],np.array(start_point)[2]), 
            Point3D(np.array(end_point)[0],np.array(end_point)[1],np.array(end_point)[2]))

        # # estimate radius for rolling ball
        # distances = points.compute_nearest_neighbor_distance()
        # avg_dist = np.mean(distances)
        # radius = 1.5 * avg_dist   
        # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(points,o3d.utility.DoubleVector([radius, radius * 2]))

        # # run the mesh- ray test
        # locations, index_ray, index_tri = mesh.ray.intersects_location(
        #     ray_origins=ray_origins,
        #     ray_directions=ray_directions)

        # # stack rays into line segments for visualization as Path3D
        # ray_visualize = trimesh.load_path(np.hstack((
        #     ray_origins,
        #     ray_origins + ray_directions)).reshape(-1, 2, 3))

        # # make mesh transparent- ish
        # mesh.visual.face_colors = [100, 100, 100, 100]

        # # create a visualization scene with rays, hits, and mesh
        # scene = trimesh.Scene([
        #     mesh,
        #     ray_visualize,
        #     trimesh.points.PointCloud(locations)])

        # # display the scene
        # scene.show()
        

=======
>>>>>>> 6d11e9be96630627fabd12365a1a8e1e05ffacd0
    def get_body_points(self, human, pos, xyz_array):
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

    def get_hand_points(self, hand, pos, xyz_array):
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
        #print(points.header.frame_id)
        #print(points.height, points.width)

        # xyz_array returns in meters
        return xyz_array[x][y]


    # def set_flags(self):
    #     parser = argparse.ArgumentParser()
    #     parser.add_argument("--num_gpu", default=op.get_gpu_number(), help="Number of GPUs.")
    #     return parser.parse_known_args()

    def set_params(self):
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
        params["face"] = False
        # params["3d"] = True
        # params['3d_views'] = 2
        return params

    def print_body_parameters(self, datum):
        print("Body keypoints: \n" + str(np.around(datum.poseKeypoints).astype(int)))
        #print("Face keypoints: \n" + str(np.around(datum.faceKeypoints).fillna(0.0).astype(int)))
        print("Left hand keypoints: \n" + str(np.around(datum.handKeypoints[0]).astype(int)))
        print("Right hand keypoints: \n" + str(np.around(datum.handKeypoints[1]).astype(int)))

    def execute(self, userdata):
        # # Flags
        # args = self.set_flags()

        # Set Params
        params = self.set_params()

        self.bridge = CvBridge()
        img_msg = rospy.wait_for_message('/xtion/rgb/image_raw',Image)
        
        # To save the depth coordinates once so that I don't have to call it again and doesnt slow everything
        points = rospy.wait_for_message('/xtion/depth_registered/points',PointCloud2)
        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(points, remove_nans=False)
        xyz_array = np.transpose(xyz_array, (1, 0, 2))


        #img_msg2 = rospy.wait_for_message('/xtion/depth_registered/image_raw',Image)
        #print(img_msg.height, img_msg.width)
        #print(img_msg2.height, img_msg2.width)

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
            self.print_body_parameters(datum)

            for i in range(human_count):
                print('=================================')
                #get_body_angle(datum.poseKeypoints[i], 'waist')
                #angle = get_body_angle(datum.poseKeypoints[i], 'left_elbow')

<<<<<<< HEAD
                head = self.get_body_points(datum.poseKeypoints[i], 'Head', xyz_array)
=======
>>>>>>> 6d11e9be96630627fabd12365a1a8e1e05ffacd0
                chest = self.get_body_points(datum.poseKeypoints[i], 'Neck', xyz_array)
                right_shoulder = self.get_body_points(datum.poseKeypoints[i], 'RShoulder', xyz_array)
                left_shoulder = self.get_body_points(datum.poseKeypoints[i], 'LShoulder', xyz_array)
                right_shoulder = self.get_body_points(datum.poseKeypoints[i], 'RShoulder', xyz_array)
                left_elbow = self.get_body_points(datum.poseKeypoints[i], 'LElbow', xyz_array)
                right_elbow = self.get_body_points(datum.poseKeypoints[i], 'RElbow', xyz_array)
                left_hand_tip = self.get_hand_points(datum.handKeypoints[0][i], 'first_finger_tip', xyz_array)
                right_hand_tip = self.get_hand_points(datum.handKeypoints[1][i], 'first_finger_tip', xyz_array)
                left_elbow_angle = self.get_elbow_angle(left_shoulder,left_elbow,left_hand_tip,'left')
                right_elbow_angle = self.get_elbow_angle(right_shoulder,right_elbow,right_hand_tip,'right')
                left_hand_tip_delta = self.get_hand_tip_delta(left_hand_tip,chest,'left')
                right_hand_tip_delta = self.get_hand_tip_delta(right_hand_tip,chest,'right')

                # Parameters that need to be satisfied in case hand is pointing based on observed data points
<<<<<<< HEAD
                # Later try to shift this functionality to is_pointing() funtion
                if ((left_elbow_angle > 120)and(abs(left_hand_tip_delta)>0.5)):
                    print('left hand pointing')
                    self.get_pointing_line(left_hand_tip_delta, head, points)
                elif ((right_elbow_angle > 120)and(abs(right_hand_tip_delta)>0.5)):
                    print('right hand pointing')
                    self.get_pointing_line(right_hand_tip_delta, head, points)
=======
                if ((left_elbow_angle > 120)and(abs(left_hand_tip_delta)>0.5)):
                    print('left hand pointing')
                elif ((right_elbow_angle > 120)and(abs(right_hand_tip_delta)>0.5)):
                    print('right hand pointing')
>>>>>>> 6d11e9be96630627fabd12365a1a8e1e05ffacd0
                else:
                    print('hand not pointing')

            cv2.imshow("Image Window", datum.cvOutputData)
            cv2.waitKey(0)

        
        except Exception as e:
            print(e)
            sys.exit(-1)

        #opWrapper.stop()
        return 'outcome1'
