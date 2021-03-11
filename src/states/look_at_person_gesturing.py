#!/usr/bin/env python
import rospy
import actionlib


from utilities import Tiago
from smach import State
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped


class LookAtPersonGesturing(State):
    def __init__(self):
        rospy.loginfo('LookAtPersonGesturing state initialized')
        
        State.__init__(self, outcomes=['outcome1','outcome2'])
        
        self.tiago = Tiago()

    def execute(self, userdata, wait=True):
        rospy.loginfo('LookAtPersonGesturing state executing')
        
        # Lift tiago's torso and set head to default
        self.tiago.look_at_person(True)

        # create the action client:
        movebase_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # wait until the action server has started up and started listening for goals
        movebase_client.wait_for_server()

        #location = rospy.get_param('/hand_gesture_approach')
        # getting approach point for person to use the same orientation to turn towards person.
        person_approach_location = rospy.get_param('/pointing_person_approach')
        # getting approach point for current table to use the same position and not move the robot base as it is already next to the table.
        table = rospy.get_param('/current_table')
        table_approach_location = table.get('location')

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose = Pose(position = Point(**table_approach_location['position']),
                                    orientation = Quaternion(**person_approach_location['orientation']))


        movebase_client.send_goal(goal)

        rospy.loginfo('GOAL SENT! o:')

        # waits for the server to finish performing the action
        if wait:
            if movebase_client.wait_for_result():
                rospy.loginfo('Goal location achieved!')
                # operator = getLocation()           
                # if operator:
                #     return get_closer_to_person(operator)
            else:
                rospy.logwarn("Couldn't reach the goal!")
        
        return 'outcome1'
