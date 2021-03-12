#!/usr/bin/env python
import rospy
import cv2
from smach import State, StateMachine


from utilities import Classify
# Tried doing this so that gazebo dooesnt eat up all the ram needed for openpose
#from utilities import GetPoseBeforeGazebo
from states import ApproachPersonPointing, ObjectDetection, GetPose, ApproachPointedObject, PointedObjectDetection, ObjectDisambiguation, LookAtPersonGesturing



# main
def main():
    rospy.init_node('state_machine')

    # Tried doing this so that gazebo dooesnt eat up all the ram needed for openpose
    #actions = GetPoseBeforeGazebo()
    #actions.execute()

    # default dataset for yolo3 is coco unless change needed for more accurate detection from a particular dataset, which is passed here.
    classify = Classify(dataset='coco')
    

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['outcome1', 'end'])
    # Open the container

    with sm:
        # Add states to the container
        # StateMachine.add('approach_person_pointing', ApproachPersonPointing(), transitions={'outcome1':'detect_objects', 'outcome2': 'detect_objects'})
        # StateMachine.add('detect_objects', ObjectDetection(classify), transitions={'outcome1':'get_pose', 'outcome2': 'get_pose'})
        # StateMachine.add('get_pose', GetPose(), transitions={'outcome1':'approach_object', 'outcome2': 'approach_object'})
        # StateMachine.add('approach_object', ApproachPointedObject(), transitions={'outcome1':'detect_pointed_object', 'outcome2': 'detect_pointed_object'})
        # StateMachine.add('detect_pointed_object', PointedObjectDetection(classify), transitions={'outcome1':'look_at_person_gesturing', 'outcome2': 'end'})
        # StateMachine.add('look_at_person_gesturing', LookAtPersonGesturing(), transitions={'outcome1':'disambiguate_objects', 'outcome2': 'disambiguate_objects'})
        StateMachine.add('disambiguate_objects', ObjectDisambiguation(), transitions={'outcome1':'end', 'outcome2': 'end'})
        sm.execute()
    
    #cv2.waitKey(0)

    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('State Machine terminated...')