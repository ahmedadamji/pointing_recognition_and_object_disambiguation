#!/usr/bin/env python
import rospy
import cv2
from smach import State, StateMachine

from utilities import ClassifyObjects, Tiago, Util, Move#, ClassifyHands
# Tried doing this so that gazebo dooesnt eat up all the ram needed for openpose
#from utilities import GetPoseBeforeGazebo
from states import ApproachPerson, PointingLocationDetection, ApproachPointedObject, PointedObjectDetection, ObjectDisambiguation, LookAtPersonForInteraction

# import sys
# sys.path.append('/tiago_ws/src/openpose/build/python')
# from openpose import pyopenpose as op

# def set_params():
#     rospy.loginfo('Setting OpenPose default parameters')
#     params = dict()
#     params["body"] = 1
#     params["number_people_max"] = 1
#     params['model_folder'] = '/tiago_ws/src/openpose/models/'
#     params['model_pose'] = 'COCO'
#     # Even tough 320x320 is dangerously accurate, it is too slow and therefore I
#     # will use the fairly accurate 320x240
#     params['net_resolution'] = '320x240' # 368x368 (multiples of 16)
#     # params['face_net_resolution'] = '160x80' # 368x368 (multiples of 16)
#     # params['hand_net_resolution'] = '160x80' # 368x368 (multiples of 16)
#     # params['flir_camera'] = True # Used when using Flir camera
#     # params['frame_undistort'] = True # Used when simultaneously using FLIR cameras and the 3-D reconstruction module so their camera parameters are read.
#     params['hand'] = True
#     params['face'] = False
#     # params["3d"] = True
#     # params['3d_views'] = 2
#     return params


# main
def main():
    rospy.init_node('state_machine')

    # params = set_params()
    
    # opWrapper = op.WrapperPython()
    # opWrapper.configure(params)
    # opWrapper.start()

    # Tried doing this so that gazebo dooesnt eat up all the ram needed for openpose
    #actions = GetPoseBeforeGazebo()
    #actions.execute()

    # default dataset for yolo3 is coco unless change needed for more accurate detection from a particular dataset, which is passed here.
    # openimages can be used which offers almost all common objects for detection
    classify_objects = ClassifyObjects(dataset='openimages')
    #creates an instance of tiago class to interact with the user and perform physical actions
    tiago = Tiago()
    # Lift tiago's torso and set head to default
    tiago.lift_torso_head_default(True)
    #creates an instance of util class to use featues such as extract attributes of objects from yaml file and transform point frames
    util = Util()
    #creates an instance of move class to move robot across the map
    move = Move()


    ## REMOVE FOLLOWING CODE WHEN RUNNING POSE DETECTION AS THESE ARE DEFAULT VALUES TO USE FOR TESTING
    intersection_point_2d = [388, 239]
    rospy.set_param('/intersection_point_2d', intersection_point_2d)
    intersection_point_3d = [0.2793646814287425, -0.004621289580974977, 2.1561762792235117]
    rospy.set_param('/intersection_point_3d', intersection_point_3d)
    intersection_point_world = [-2.195603797301315, -9.08019820397351, 1.0689875402030786]
    rospy.set_param('/intersection_point_world', intersection_point_world)
    person_head_world_coordinate = [-1.402642251022807, -8.916499175910454, 1.792557797900345]
    rospy.set_param('/person_head_world_coordinate', person_head_world_coordinate)
    radius_of_pointing = 0.18
    rospy.set_param('/radius_of_pointing', radius_of_pointing)
    

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['outcome1', 'end'])
    # Open the container

    with sm:
        # Add states to the container
        #StateMachine.add('approach_person', ApproachPerson(classify_objects, tiago, util, move), transitions={'outcome1':'detect_pointing_location', 'outcome2': 'detect_pointing_location'})
        #StateMachine.add('detect_pointing_location', PointingLocationDetection(tiago, util), transitions={'outcome1':'approach_object', 'outcome2': 'end'})
        StateMachine.add('approach_object', ApproachPointedObject(tiago, util, move), transitions={'outcome1':'detect_pointed_object', 'outcome2': 'end'})
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