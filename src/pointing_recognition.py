#!/usr/bin/env python
import rospy
from smach import State, StateMachine


from utilities import Tiago, Classify
# Tried doing this so that gazebo dooesnt eat up all the ram needed for openpose
#from utilities import GetPoseBeforeGazebo
from states import ApproachPointingGirl, ObjectDetection, GetPose


# main
def main():
    rospy.init_node('pointing_recognition')

    # Tried doing this so that gazebo dooesnt eat up all the ram needed for openpose
    #actions = GetPoseBeforeGazebo()
    #actions.execute()

    tiago = Tiago()
    classify = Classify(dataset='coco')
    
    # tiago.lift_torso_head_default(True)

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['outcome1', 'end'])
    # Open the container

    with sm:
        # Add states to the container
        StateMachine.add('approach_pointing_girl', ApproachPointingGirl(), transitions={'outcome1':'detect_objects', 'outcome2': 'detect_objects'})
        StateMachine.add('detect_objects', ObjectDetection(classify), transitions={'outcome1':'get_pose', 'outcome2': 'get_pose'})
        StateMachine.add('get_pose', GetPose(), transitions={'outcome1':'end', 'outcome2': 'end'})
        sm.execute()

    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('State Machine terminated...')