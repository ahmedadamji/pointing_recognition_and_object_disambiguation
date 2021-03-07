#!/usr/bin/env python
import rospy
import actionlib


from utilities import Tiago
from smach import State
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped


class ApproachPersonPointing(State):
    def __init__(self):
        rospy.loginfo('ApproachPersonPointing state initialized')
        
        State.__init__(self, outcomes=['outcome1','outcome2'])
        
        self.tiago = Tiago()

    def execute(self, userdata, wait=True):
        rospy.loginfo('ApproachPersonPointing state executing')
        
        # Lift tiago's torso and set head to default
        self.tiago.lift_torso_head_default(True)

        # create the action client:
        movebase_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # wait until the action server has started up and started listening for goals
        movebase_client.wait_for_server()

        location = rospy.get_param('/pointing_person_approach')

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose = Pose(position = Point(**location['position']),
                                    orientation = Quaternion(**location['orientation']))


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
