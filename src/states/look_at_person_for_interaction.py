#!/usr/bin/env python
import rospy
import actionlib


from utilities import Tiago
from smach import State
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped


class LookAtPersonForInteraction(State):
    def __init__(self):
        rospy.loginfo('LookAtPersonForInteraction state initialized')
        
        State.__init__(self, outcomes=['outcome1','outcome2'])
        
        #creates an instance of tiago class to interact with the user and perform physical actions
        self.tiago = Tiago()
        #creates an instance of move class to move robot across the map
        self.move = Move()

    def execute(self, userdata, wait=True):
        rospy.loginfo('LookAtPersonForInteraction state executing')

        self.tiago.talk("I will now look towards the person to recognise hand gestures, to gather responses for disambiguation." )
        
        # Lift tiago's torso and set head to default
        self.tiago.look_at_person(True)


        #location = rospy.get_param('/hand_gesture_approach')
        # getting approach point for person to use the same orientation to turn towards person.
        person_approach_location = rospy.get_param('/pointing_person_approach')
        # getting approach point for current table to use the same position and not move the robot base as it is already next to the table.
        table = rospy.get_param('/current_table')
        table_approach_location = table.get('approach_location')

        location = {
            'position': table_approach_location['position']
            'orientation': person_approach_location['orientation']
        }

        # Sending Move class the location to move to, and stores result in movebase
        movebase = self.move.move_base(location)
        if movebase == True:
            self.tiago.talk("I am now looking at the person pointing, to gather responses" )
        else:
            # INSERT HERE THE ACTION IF GOAL NOT ACHIEVED
            self.tiago.talk("I have not been able to reach the goal location" )

        
        return 'outcome1'
