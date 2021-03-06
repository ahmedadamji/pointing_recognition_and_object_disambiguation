#!/usr/bin/env python
import rospy
import actionlib
import tf
import numpy as np
import cv2

from smach import State

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
#from pointing_recognition.msg import IntersectionData



class ApproachPointedObject(State):
    def __init__(self, interaction, util, move):
        #rospy.loginfo("ApproachPointedObject state initialized")
        State.__init__(self, outcomes=["outcome1","outcome2"])

        #creates an instance of interaction class to interact with the user
        self.interaction = interaction
        #creates an instance of util class to transform point frames
        self.util = util
        #creates an instance of move class to move robot across the map and perform physical actions
        self.move = move



    def get_table(self, intersection_point_world):
        current_table = rospy.get_param("/current_table")
        table_name = current_table.get("name")
        print("current table name: " + table_name)
        return current_table



    def approach_table(self, table, wait=True):

        print("Approaching selected table")

        location = table.get("approach_location")
        # Sending Move class the location to move to, and stores result in movebase
        movebase = self.move.move_base(location)
        if movebase == True:
            print("Reached the goal location" )
        else:
            # INSERT HERE THE ACTION IF GOAL NOT ACHIEVED
            self.interaction.talk("Sorry but I have not been able to reach close to the " + str(table.get("name")) )
            return False

        # Sets robot pose to check table
        self.move.check_table(True)
        return True
        


    def execute(self, userdata, wait=True):

        # Collects the details of tables in the environment from the util class and saves in self.tables
        self.tables = self.util.tables

        rospy.loginfo("ApproachPointedObject state executing")

        # intersection_point = rospy.wait_for_message("/intersection_point", IntersectionData)
        # print intersection_point.intersection_point_2d
        # print intersection_point.intersection_point_3d
        # intersection_point_2d = rospy.get_param("/intersection_point_2d")
        # intersection_point_3d = rospy.get_param("/intersection_point_3d")
        intersection_point_world = rospy.get_param("/intersection_point_world")
        # print intersection_point_2d
        # print intersection_point_3d


        table = self.get_table(intersection_point_world)
        # saving selected table name to use its location while looking at person gesturing 
        rospy.set_param("/current_table", table)
        self.interaction.talk("I will now approach close to the " + str(table.get("name"))+ " to scan for objects")
        if not self.approach_table(table, wait):
            return "outcome2"
        
        return "outcome1"
