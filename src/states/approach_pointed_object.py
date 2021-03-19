#!/usr/bin/env python
import rospy
import actionlib
import tf
import numpy as np
import cv2

from smach import State
from utilities import Tiago, Util

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
#from pointing_recognition.msg import IntersectionData



class ApproachPointedObject(State):
    def __init__(self):
        rospy.loginfo('ApproachPointedObject state initialized')
        State.__init__(self, outcomes=['outcome1','outcome2'])

        #creates an instance of tiago class to interact with the user and perform physical actions
        self.tiago = Tiago()
        #creates an instance of util class to transform point frames
        self.util = Util()
        # Collects the details of tables in the environment from the util class and saves in self.tables
        self.tables = self.util.tables


    def get_table(self, intersection_point_world):
        #tables = rospy.get_param('/tables')
        for table_id in range(0, len(self.tables)):
            table_name = self.tables[table_id].get('name')
            cuboid = self.tables[table_id].get('cuboid')
            cuboid_max = np.array(cuboid['max_xyz'])
            cuboid_min = np.array(cuboid['min_xyz'])
            print cuboid_max
            print cuboid_min
            print intersection_point_world
            if ((cuboid_min[0] <= intersection_point_world[0] <= cuboid_max[0]) and (cuboid_min[1] <= intersection_point_world[1] <= cuboid_max[1])  and (cuboid_min[2] <= intersection_point_world[2] <= cuboid_max[2])):
                print table_name + ' is being pointed at'
                return self.tables[table_id]
        ## RETURN OUTCOME 2 AFTER ERROR MESSAGE SAYING NO TABLE WAS FOUND BEING POINTED AT
        print 'Sorry, no table was found being pointed at'


    def approach_table(self, table, wait=True):
        rospy.loginfo('Approaching table selected')
        
        # create the action client:
        movebase_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # wait until the action server has started up and started listening for goals
        movebase_client.wait_for_server()

        location = table.get('approach_location')


        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose = Pose(position = Point(**location['position']),
                                    orientation = Quaternion(**location['orientation']))


        movebase_client.send_goal(goal)

        rospy.loginfo('GOAL SENT! o:')

        # Sets robot pose to check table
        self.tiago.check_table(True)

        # waits for the server to finish performing the action
        if wait:
            if movebase_client.wait_for_result():
                rospy.loginfo('Goal location achieved!')
                # operator = getLocation()           
                # if operator:
                #     return get_closer_to_person(operator)
            else:
                rospy.logwarn("Couldn't reach the goal!")
        


    def execute(self, userdata, wait=True):

        rospy.loginfo('ApproachPointedObject state executing')

        # intersection_point = rospy.wait_for_message('/intersection_point', IntersectionData)
        # print intersection_point.intersection_point_2d
        # print intersection_point.intersection_point_3d
        # intersection_point_2d = rospy.get_param("/intersection_point_2d")
        # intersection_point_3d = rospy.get_param("/intersection_point_3d")
        intersection_point_world = rospy.get_param("/intersection_point_world")
        # print intersection_point_2d
        # print intersection_point_3d

        self.tiago.talk("I am now checking which table the pointed object lies on" )

        table = self.get_table(intersection_point_world)
        # saving selected table name to use its location while looking at person gesturing 
        rospy.set_param('/current_table', table)
        self.tiago.talk("I will now approach " + str(table.get('name')))
        self.approach_table(table, wait)
        
        return 'outcome1'
