#!/usr/bin/env python
import rospy
import cv2
from smach import State, StateMachine


from utilities import Tiago, Classify
# Tried doing this so that gazebo dooesnt eat up all the ram needed for openpose
#from utilities import GetPoseBeforeGazebo
from states import ApproachPersonPointing, ObjectDetection, GetPose, ApproachPointedObject



# main
def main():
    rospy.init_node('state_machine')

    # Tried doing this so that gazebo dooesnt eat up all the ram needed for openpose
    #actions = GetPoseBeforeGazebo()
    #actions.execute()

    tiago = Tiago()
    classify = Classify(dataset='coco') # default dataset for yolo3 is coco unless change needed for more accurate detection from a distance, which is passed here.
    
    tiago.lift_torso_head_default(True)

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['outcome1', 'end'])
    # Open the container

    with sm:
        # Add states to the container
        StateMachine.add('approach_pointing_girl', ApproachPersonPointing(), transitions={'outcome1':'detect_objects', 'outcome2': 'detect_objects'})
        StateMachine.add('detect_objects', ObjectDetection(classify), transitions={'outcome1':'get_pose', 'outcome2': 'get_pose'})
        StateMachine.add('get_pose', GetPose(), transitions={'outcome1':'approach_object', 'outcome2': 'approach_object'})
        StateMachine.add('approach_object', ApproachPointedObject(), transitions={'outcome1':'detect_objects_again', 'outcome2': 'detect_objects_again'})
        StateMachine.add('detect_objects_again', ObjectDetection(classify), transitions={'outcome1':'end', 'outcome2': 'end'})
        sm.execute()
    
    cv2.waitKey(0)

    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('State Machine terminated...')