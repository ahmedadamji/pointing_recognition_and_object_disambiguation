#!/usr/bin/env python
import rospy
import actionlib
import math


from smach import State
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped


class LookAtPersonForInteraction(State):
    def __init__(self, interaction, move):
        #rospy.loginfo("LookAtPersonForInteraction state initialized")
        
        State.__init__(self, outcomes=["outcome1","outcome2"])
        
        #creates an instance of interaction class to interact with the user
        self.interaction = interaction
        #creates an instance of move class to move robot across the map and perform physical actions
        self.move = move

    def execute(self, userdata, wait=True):
        rospy.loginfo("LookAtPersonForInteraction state executing")

        self.interaction.talk("I will now turn towards you to ask questions" )
        
        # Lift tiago's torso and set head to default
        self.move.look_at_person(True)

        # getting the person's head world coordinate, to turn towards them for interaction
        person_head_world_coordinate = rospy.get_param("/person_head_world_coordinate")
        robot_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped).pose.pose
        robot_world_coordinate = [robot_pose.position.x,robot_pose.position.y]
        delta_y = person_head_world_coordinate[1] - robot_world_coordinate[1]
        delta_x = person_head_world_coordinate[0] - robot_world_coordinate[0]
        theta = -math.atan2(delta_y,delta_x)/(math.pi/180)
        self.move.rotate_around_base(theta)


        # #location = rospy.get_param("/hand_gesture_approach")
        # # getting approach point for person to use the same orientation to turn towards person.
        # pointing_person_approach_orientation = rospy.get_param("/pointing_person_approach_orientation")
        # pointing_person_approach_orientation = { "x": pointing_person_approach_orientation[0], "y": pointing_person_approach_orientation[1], "z": pointing_person_approach_orientation[2], "w": pointing_person_approach_orientation[3] }
        # # getting approach point for current table to use the same position and not move the robot base as it is already next to the table.
        # table = rospy.get_param("/current_table")
        # table_approach_location = table.get("approach_location")

        # location = {
        #     "position": table_approach_location["position"],
        #     "orientation": pointing_person_approach_orientation
        # }

        # # Sending Move class the location to move to, and stores result in movebase
        # movebase = self.move.move_base(location)
        # if movebase == True:
        #     #self.interaction.talk("I am now looking at the person pointing, to gather responses" )
        # else:
        #     # INSERT HERE THE ACTION IF GOAL NOT ACHIEVED
        #     self.interaction.talk("I have not been able to look towards you" )

        
        return "outcome1"
