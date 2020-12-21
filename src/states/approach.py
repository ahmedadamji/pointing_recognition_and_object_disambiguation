#!/usr/bin/env python
import rospy
import actionlib

from smach import State
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped


class ApproachPointingGirl(State):
    def __init__(self):
        State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):

        # create the action client:
        movebase_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # wait until the action server has started up and started listening for goals
        movebase_client.wait_for_server()

        location = rospy.get_param('/girl_pointing')

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose = Pose(position = Point(**location['position']),
                                    orientation = Quaternion(**location['orientation']))


        movebase_client.send_goal(goal)

        rospy.loginfo('GOAL SENT! o:')

        # waits for the server to finish performing the action
        if movebase_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
            # operator = getLocation()           
            # if operator:
            #     return get_closer_to_person(operator)
        else:
            rospy.logwarn("Couldn't reach the goal!")
        
        return 'outcome1'
