#!/usr/bin/env python
import rospy
from smach import State, StateMachine


from utilities import Tiago
from states import ApproachPointingGirl
from states import GetPose


# main
def main():
    rospy.init_node('pointing_recognition')

    tiago = Tiago()

    tiago.lift_torso_head_default()


    # Create a SMACH state machine
    sm = StateMachine(outcomes=['outcome1', 'end'])
    # Open the container

    with sm:
        # Add states to the container
        StateMachine.add('approach_pointing_girl', ApproachPointingGirl(), transitions={'outcome1':'get_pose', 'outcome2': 'get_pose'})
        StateMachine.add('get_pose', GetPose(), transitions={'outcome1':'end', 'outcome2': 'end'})
        sm.execute()

    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('State Machine terminated...')