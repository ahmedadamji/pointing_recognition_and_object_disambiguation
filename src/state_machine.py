#!/usr/bin/env python
import rospy
import cv2
from smach import State, StateMachine

from utilities import ClassifyObjects, Tiago, Util, Move#, ClassifyHands
# Tried doing this so that gazebo dooesnt eat up all the ram needed for openpose
#from utilities import GetPoseBeforeGazebo
from states import ApproachPerson, PointingLocationDetection, ApproachPointedObject, PointedObjectDetection, ObjectDisambiguation, LookAtPersonForInteraction



# main
def main():
    rospy.init_node('state_machine')

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
        StateMachine.add('detect_pointing_location', PointingLocationDetection(tiago, util), transitions={'outcome1':'approach_object', 'outcome2': 'approach_object'})
        StateMachine.add('approach_object', ApproachPointedObject(tiago, util, move), transitions={'outcome1':'detect_pointed_object', 'outcome2': 'detect_pointed_object'})
        StateMachine.add('detect_pointed_object', PointedObjectDetection(classify_objects, tiago, util), transitions={'outcome1':'look_at_person_for_interaction', 'outcome2': 'end'})
        StateMachine.add('look_at_person_for_interaction', LookAtPersonForInteraction(tiago, move), transitions={'outcome1':'disambiguate_objects', 'outcome2': 'disambiguate_objects'})
        StateMachine.add('disambiguate_objects', ObjectDisambiguation(tiago, move), transitions={'outcome1':'end', 'outcome2': 'end'})
        sm.execute()
    
    #cv2.waitKey(0)

    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('State Machine terminated...')