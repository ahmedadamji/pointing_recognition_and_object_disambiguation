#!/usr/bin/env python
import rospy
import cv2
from smach import State, StateMachine

from utilities import ClassifyObjects, Tiago, Util, Move#, ClassifyHands
# Tried doing this so that gazebo dooesnt eat up all the ram needed for openpose
#from utilities import GetPoseBeforeGazebo
from states import ApproachPerson, PointingLocationDetection, ApproachPointedObject, PointedObjectDetection, ObjectDisambiguation, LookAtPersonForInteraction

import sys
sys.path.append('/tiago_ws/src/openpose/build/python')
from openpose import pyopenpose as op

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


# main
def main():
    rospy.init_node('state_machine')

    params = set_params()
    
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()
    print('a')

    # Tried doing this so that gazebo dooesnt eat up all the ram needed for openpose
    #actions = GetPoseBeforeGazebo()
    #actions.execute()

    # default dataset for yolo3 is coco unless change needed for more accurate detection from a particular dataset, which is passed here.
    classify_objects = ClassifyObjects(dataset='coco')
    #creates an instance of tiago class to interact with the user and perform physical actions
    tiago = Tiago()
    #creates an instance of util class to use featues such as extract attributes of objects from yaml file and transform point frames
    util = Util()
    #creates an instance of move class to move robot across the map
    move = Move()
    

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['outcome1', 'end'])
    # Open the container

    with sm:
        # Add states to the container
        StateMachine.add('approach_person', ApproachPerson(classify_objects, tiago, util, move), transitions={'outcome1':'detect_pointing_location', 'outcome2': 'detect_pointing_location'})
        StateMachine.add('detect_pointing_location', PointingLocationDetection(opWrapper, tiago, util), transitions={'outcome1':'approach_object', 'outcome2': 'approach_object'})
        StateMachine.add('approach_object', ApproachPointedObject(tiago, util, move), transitions={'outcome1':'detect_pointed_object', 'outcome2': 'detect_pointed_object'})
        StateMachine.add('detect_pointed_object', PointedObjectDetection(classify_objects, tiago, util), transitions={'outcome1':'look_at_person_for_interaction', 'outcome2': 'end'})
        StateMachine.add('look_at_person_for_interaction', LookAtPersonForInteraction(tiago, move), transitions={'outcome1':'disambiguate_objects', 'outcome2': 'disambiguate_objects'})
        StateMachine.add('disambiguate_objects', ObjectDisambiguation(tiago, util), transitions={'outcome1':'end', 'outcome2': 'end'})
        sm.execute()
    
    #cv2.waitKey(0)

    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('State Machine terminated...')