#!/usr/bin/env python
import rospy
import actionlib
import tf
import numpy as np

from smach import State
from utilities import Tiago

from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from pointing_recognition.msg import IntersectionData


import cv2


class ApproachPointedObject(State):
    def __init__(self):
        rospy.loginfo('ApproachPointedObject state initialized')
        State.__init__(self, outcomes=['outcome1','outcome2'])
        self.transformer = tf.TransformListener()
        self.tiago = Tiago()

    
    def transform_from_camera_frame_to_world_frame(self, camera_point):

        depth_points = rospy.wait_for_message('xtion/depth_registered/points', PointCloud2)

        self.transformer.waitForTransform('xtion_rgb_optical_frame', 'map', depth_points.header.stamp, rospy.Duration(2.0))

        intersection_point = PointStamped()
        intersection_point.header = depth_points.header
        intersection_point.point = Point(*camera_point)

        person_point = self.transformer.transformPoint('map', intersection_point)
        #print person_point
        tf_point = person_point.point
        intersection_point_world = np.array([tf_point.x, tf_point.y, tf_point.z])
        return intersection_point_world

    def find_table_id(self, intersection_point_world):
        #tables = rospy.get_param('/tables')
        # Later make this loop for each table, refer p1_server in lasr scirock for this
        cuboid = rospy.get_param('/tables/' + 'table0' + '/cuboid')
        cuboid_max = np.array(cuboid['max_xyz'])
        cuboid_min = np.array(cuboid['min_xyz'])
        print cuboid_max
        print cuboid_min
        print intersection_point_world
        if ((cuboid_min[0] <= intersection_point_world[0] <= cuboid_max[0]) and (cuboid_min[1] <= intersection_point_world[1] <= cuboid_max[1])  and (cuboid_min[2] <= intersection_point_world[2] <= cuboid_max[2])):
            print 'table0' + ' is the man'
            return 'table0'


        # Pose(position = Point(**tables['position']), orientation = Quaternion(**tables['orientation']))


    def approach_table(self, table, wait=True):
        rospy.loginfo('Approaching table selected')
        
        # create the action client:
        movebase_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # wait until the action server has started up and started listening for goals
        movebase_client.wait_for_server()

        location = rospy.get_param('/tables/' + table + '/location')

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
        
        self.tiago.check_table(True)


    def execute(self, userdata, wait=True):

        rospy.loginfo('ApproachPointedObject state executing')

        # intersection_point = rospy.wait_for_message('/intersection_point', IntersectionData)
        # print intersection_point.intersection_point_2d
        # print intersection_point.intersection_point_3d
        intersection_point_2d = rospy.get_param("/intersection_point_2d")
        intersection_point_3d = rospy.get_param("/intersection_point_3d")
        # print intersection_point_2d
        # print intersection_point_3d
        intersection_point_world = self.transform_from_camera_frame_to_world_frame(intersection_point_3d)

        table = self.find_table_id(intersection_point_world)
        self.approach_table(table, wait)

        cv2.waitKey(0)
        
        return 'outcome1'
