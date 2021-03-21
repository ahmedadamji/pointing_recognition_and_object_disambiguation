#!/usr/bin/env python
import rospy
import actionlib

import sys
sys.path.append('/tiago_ws/src/openpose/build/python')
from openpose import pyopenpose as op


from pointing_recognition.srv import OpenPoseKeypoints


def print_body_parameters(datum):
    rospy.loginfo('Printing Body Parameters')
    print("Body keypoints: \n" + str(np.around(datum.poseKeypoints).astype(int)))
    #print("Face keypoints: \n" + str(np.around(datum.faceKeypoints).fillna(0.0).astype(int)))
    print("Left hand keypoints: \n" + str(np.around(datum.handKeypoints[0]).astype(int)))
    print("Right hand keypoints: \n" + str(np.around(datum.handKeypoints[1]).astype(int)))

def set_params():
    rospy.loginfo('Setting OpenPose default parameters')
    params = dict()
    params["body"] = 1
    params["number_people_max"] = 1
    params['model_folder'] = '/tiago_ws/src/openpose/models/'
    params['model_pose'] = 'BODY_25'
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

def get_body_points_3d(human, pos, xyz_array):
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
    return get_depth(pnt, xyz_array)

def get_hand_points_3d(hand, pos, xyz_array):
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
    return get_depth(pnt, xyz_array)

def get_depth(pnt, xyz_array):
    #rospy.loginfo('Requesting depth data at requested pixel')
    # Gets the z distance from tiago to the object at pixel x,y in the camera image.
    # x and y are in pixels
    x = pnt[0]
    y = pnt[1]

def get_openpose_keypoints(xyz_array):

    try:
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    try:

        # Process Image
        datum = op.Datum()
        datum.cvInputData = cv_image
        self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))


        # Display Image And Print Body Keypoints
        human_count = len(datum.poseKeypoints)
        print('Number of humans in frame: {}'.format(human_count))
        print_body_parameters(datum)
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

            # RETURN THESE KEYPOINTS

    
    except Exception as e:
        print(e)
        sys.exit(-1)

def openpose_server():
    rospy.init_node('openpose_detection')

    global opWrapper
    
    params = set_params()
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()


    serv = rospy.Service('openpose_detection', OpenPose, get_openpose_keypoints)


    rospy.loginfo('openpose_detection service initialised')
    rospy.spin()

if __name__ == '__main__':
    openpose_server()
