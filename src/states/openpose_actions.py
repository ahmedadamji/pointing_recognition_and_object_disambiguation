#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from sensor_msgs.srv import SetCameraInfo
import sensor_msgs.point_cloud2 as pc2

from pointing_recognition.msg import IntersectionData

from geometry_msgs.msg import PoseStamped

from smach import State
from utilities import Tiago, Util

import sys
sys.path.append('/tiago_ws/src/openpose/build/python')
from openpose import pyopenpose as op

import cv2
import math
import numpy as np
import ros_numpy
import argparse
import open3d as o3d

import sympy
from sympy import Point2D, Point3D
from sympy.abc import L
from sympy.geometry import Line2D, Line3D, Segment3D


# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

class GetPose(State):
    def __init__(self):
        rospy.loginfo('GetPose state initialized')
        State.__init__(self, outcomes=['outcome1','outcome2'])
        
        self.intersection_point_pub = rospy.Publisher('/intersection_point', IntersectionData)
        self.msg_to_send = IntersectionData()
        
        self.bridge = CvBridge()
        
        #creates an instance of tiago class to interact with the user
        self.tiago = Tiago()
        #creates an instance of util class to transform point frames
        self.util = Util()
    
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
        rospy.loginfo('Calculating hand tip delta')
        #comparing the x coordinate of chest and hand tip
        handtipdelta = hand_tip[0] - chest[0]
        rospy.loginfo('%s handtipdelta:%f'%(hand, handtipdelta))
        return handtipdelta

    def normalize(self, array):
        #rospy.loginfo('normalizing recieved array')
        normalized = array/np.linalg.norm(array, axis = 0)
        return normalized

    # def GetMeshDepthAtPoint(self, ICameraIntrinsics depthIntrinsics, depth_points, Point3D point, bool undistort):
    #     Point2D depthSpacePoint = self.ToPixelSpace(point, undistort)

    #     x = int (math.round(depthSpacePoint.x))
    #     y = int (math.round(depthSpacePoint.y))
    #     if ((x < 0) or (x >= depth_points.width) or (y < 0) or (y >= depth_points.height)):
    #         return float('NaN')

    #     byteOffset = int ((y * depth_points.stride) + (x * 2))
    #     depth = int(depth_points.ReadBytes(2, byteOffset))
    #     if (depth == 0):
    #         return float('NaN')

    #     return float (depth / 1000)

    # def ToPixelSpace(self, Point3D pt, bool distort):
    #     # X points in the depth dimension. Y points to the left, and Z points up.
    #     pixelPt = Point2D((-pt.y / pt.x), (-pt.z / pt.x))
    #     if (distort):
    #         this.DistortPoint(pixelPt, out pixelPt)

    #     tmp = Point3D(pixelPt.x, pixelPt.y, 1.0)
    #     tmp = tmp.TransformBy(this.transform)
    #     return Point2D(tmp.x, tmp.y)

    def project_depth_array_to_2d_image_pixels(self, point_3d):
        rospy.loginfo('projecting depth array to 2d image pixels')
        camera_info = rospy.wait_for_message('/xtion/rgb/camera_info', CameraInfo)
        depth_array = np.array([point_3d[0], point_3d[1], point_3d[2], 1])
        uvw = np.dot(np.array(camera_info.P).reshape((3, 4)), depth_array.transpose()).transpose()
        x = int(uvw[0] / uvw[2])
        y = int(uvw[1] / uvw[2])

        return x,y

    def intersect_line_with_depth_mesh(self, xyz_array, skipFactor, maxDistance, start_point, end_point):
        # SECOND ATTEMPT AT FINDING COLLISION WITH MESH -->
        delta = skipFactor * self.normalize(end_point - start_point)
        maxSteps = int(maxDistance / (np.linalg.norm(delta)))
        hypothesis_point_3d = start_point
        for i in range(maxSteps):
            hypothesis_point_3d += delta
            # Get 2D xy coordinate of the hypothesis point
            hypothesis_point_2d = np.array(self.project_depth_array_to_2d_image_pixels(hypothesis_point_3d))


            # get the mesh distance at the extended point
            ## Fix this code as it doesnt make sense -->
            # float meshDistance = GetMeshDepthAtPoint(depthIntrinsics, depth_points, hypothesis_point_3d, undistort);
            meshDistance = xyz_array[hypothesis_point_2d[0]][hypothesis_point_2d[1]][2]
            # if the mesh distance is less than the distance to the point we've hit the mesh
            # can do so that in the selected pixel space, I can compare the depth, if the
            # depth of the pointcloud is less then it is intersecting
            # or i can just check if the point is inside the box selected
            if (not(math.isnan(meshDistance)) and (meshDistance < hypothesis_point_3d[2])):
                print(hypothesis_point_2d)
                return hypothesis_point_3d, hypothesis_point_2d
                ## TEST HERE IF THERE IS OBSTRUCTION TO POINTING LINE< WILL IT GIVE ERROR< ALSO MAKE IT SUCH THAT THE INTERSECTION IS ONLY CHECKED WITHIN A RADIUS
                ## OF THE POINTING LINE TO ELIMINATE PROBLEMS WITH FAR AWAY OBJECTS OVERLAPPING WITH MESH, CHECK PERFORMANCE OF THIS AS WELL AND INCLUDE RESULTS OF BOTH
                ## BEFORE AND AFTER IN REPORT
        

        ## FIRST ATTEMPTH AT FINDING COLLISION WITH MESH -->
        # exit = False
        # for x in range(640 - tip[0] - 20):
        #     for y in range(480 - tip[1] - 20):
        #         if math.isnan((xyz_array[x+tip[0]+20][y+tip[1]+20])[0]):
        #             point = Point3D(0,0,0)
        #         else:
        #             recorded_point = np.array(xyz_array[x+tip[0]+20][y+tip[1]+20])
        #             point = Point3D(recorded_point[0],recorded_point[1],recorded_point[2])
                
        #         intersection = np.array(line.intersection(point))
        #         if intersection.size > 0:
        #             print(line.intersection(point))
        #             exit = True
        #         if(exit):
        #             break
        #     if(exit):
        #         break



    def get_pointing_line(self, hand_tip, head, xyz_array, hand, open_pose_output_image, maxDistance = 5, skipFactor = 0.05):
        rospy.loginfo('calucating the line of pointing')
        
        # https://github.com/mikedh/trimesh/blob/master/examples/ray.py
        # https://github.com/mikedh/trimesh/issues/211
        # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        # https://github.com/microsoft/psi
        # https://github.com/microsoft/psi/blob/master/Sources/Calibration/Microsoft.Psi.Calibration/CalibrationExtensions.cs
        # https://github.com/microsoft/psi/blob/dd60dbbc2651ed2a515b9d0b1c2ea2d2adfc2e25/Sources/Calibration/Microsoft.Psi.Calibration/CameraIntrinsics.cs#L101

        # find a way to get the depth mesh from the rgbd camera

        # Finding tip location to determine which area of image to detect for overlap
        #tip = [int(hand[8][0]), int(hand[8][1])]

        direction = self.normalize(hand_tip - head)
        # print(np.shape(direction))

        # Do not start the line right at the hand-tip to avoid having an intersection with the mesh around the hand.
        start_point = hand_tip + (direction*0.05)
        end_point = hand_tip + (direction*0.15)

        start_point_3d = np.array(start_point)
        end_point_3d = np.array(end_point)

        # line = Line3D(start_point_3d, end_point_3d)


        start_point_2d = np.array(self.project_depth_array_to_2d_image_pixels(start_point_3d))
        end_point_2d = np.array(self.project_depth_array_to_2d_image_pixels(end_point_3d))

        # print(start_point_3d, start_point_2d, end_point_3d, end_point_2d)

        # rospy.loginfo('displaying pointing line')
        # cv2.line(open_pose_output_image, (start_point_2d[0],start_point_2d[1]), (end_point_2d[0],end_point_2d[1]), (0,0,255), 2)
        # cv2.imshow("Pointing Line Results", open_pose_output_image)
        # cv2.waitKey(5000)
        # comment this later to stop state machine automatically and move past this code
        
        #cv2.waitKey(0)

        intersection_point_3d, intersection_point_2d = self.intersect_line_with_depth_mesh(xyz_array, skipFactor, maxDistance, start_point, end_point)
        
        rospy.loginfo('displaying extended pointing line towards mesh')
        # Stores extended line upto mesh
        cv2.line(open_pose_output_image, (start_point_2d[0],start_point_2d[1]), (intersection_point_2d[0],intersection_point_2d[1]), (255,255,0), 1)
        # Stores box around overlapping point
        box_start_point = (intersection_point_2d[0]-25),(intersection_point_2d[1]-25)
        box_end_point = intersection_point_2d[0]+25,intersection_point_2d[1]+25
        cv2.rectangle(open_pose_output_image, box_start_point, box_end_point, (0,0,255), 1)
        # Plots all figures on top of an opencv image of openpose keypoints
        cv2.imshow("Pointing Line Results", open_pose_output_image)
        cv2.waitKey(5000)
        
        # self.msg_to_send.intersection_point_2d = intersection_point_2d
        # self.msg_to_send.intersection_point_3d = intersection_point_3d

        # self.intersection_point_pub.publish(self.msg_to_send)

        rospy.set_param('/intersection_point_2d', [intersection_point_2d[0].item(), intersection_point_2d[1].item()])
        rospy.set_param('/intersection_point_3d', [intersection_point_3d[0].item(), intersection_point_3d[1].item(), intersection_point_3d[2].item()])
        #rospy.set_param('/start_point_3d', [start_point_3d[0].item(), start_point_3d[1].item(), start_point_3d[2].item()])

        #cv2.waitKey(0)

    
    
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
        params["face"] = False
        # params["3d"] = True
        # params['3d_views'] = 2
        return params

    def print_body_parameters(self, datum):
        rospy.loginfo('Printing Body Parameters')
        print("Body keypoints: \n" + str(np.around(datum.poseKeypoints).astype(int)))
        #print("Face keypoints: \n" + str(np.around(datum.faceKeypoints).fillna(0.0).astype(int)))
        print("Left hand keypoints: \n" + str(np.around(datum.handKeypoints[0]).astype(int)))
        print("Right hand keypoints: \n" + str(np.around(datum.handKeypoints[1]).astype(int)))

    def execute(self, userdata):
        rospy.loginfo('GetPose state executing')
        # # Flags
        # args = self.set_flags()

        # Set Params
        params = self.set_params()

        img_msg = rospy.wait_for_message('/xtion/rgb/image_raw',Image)
        
        # To save the depth coordinates once so that I don't have to call it again and doesnt slow everything
        ## MAKE XYZ_ARRAY A GLOBAL VARIABLE
        depth_points = rospy.wait_for_message('/xtion/depth_registered/points',PointCloud2)
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
            open_pose_output_image = datum.cvOutputData
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

                # Parameters that need to be satisfied in case hand is pointing based on observed data points
                # Later try to shift this functionality to is_pointing() funtion
                if ((left_elbow_angle > 120)and(abs(left_hand_tip_delta)>0.5)):
                    print('left hand pointing')
                    self.get_pointing_line(left_hand_tip, head, xyz_array, datum.handKeypoints[0][i], open_pose_output_image, 1, 0.05)

                    # Saving world coordinate for head for use during disambiguation in reference to user location
                    person_head_world_coordinate = self.util.transform_from_camera_frame_to_world_frame(head)
                    rospy.set_param('/person_head_world_coordinate', [person_head_world_coordinate[0], person_head_world_coordinate[1], person_head_world_coordinate[2]])

                    return 'outcome1'
                    
                elif ((right_elbow_angle > 120)and(abs(right_hand_tip_delta)>0.5)):
                    print('right hand pointing')
                    self.get_pointing_line(right_hand_tip, head, xyz_array, datum.handKeypoints[1][i], open_pose_output_image, 1, 0.05)

                    # Saving world coordinate for head for use during disambiguation in reference to user location
                    person_head_world_coordinate = self.util.transform_from_camera_frame_to_world_frame(head)
                    rospy.set_param('/person_head_world_coordinate', [person_head_world_coordinate[0].item(), person_head_world_coordinate[1].item(), person_head_world_coordinate[2].item()])

                    return 'outcome1'

                else:
                    print('hand not pointing')

                

        
        except Exception as e:
            print(e)
            sys.exit(-1)

        
        # To destroy cv2 window at the end of state
        cv2.destroyAllWindows()

        #opWrapper.stop()
        
        return 'outcome1'
