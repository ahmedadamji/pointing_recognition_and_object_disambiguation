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
from sensor_msgs.msg import Image, PointCloud2, RegionOfInterest, PointField, CameraInfo
from nav_msgs.msg import OccupancyGrid
from pointing_recognition.msg import IntersectionData
from cv_bridge import CvBridge, CvBridgeError

import cv2


class ApproachPointedObject(State):
    def __init__(self):
        rospy.loginfo('ApproachPointedObject state initialized')
        State.__init__(self, outcomes=['outcome1','outcome2'])
        self.transformer = tf.TransformListener()
        self.tiago = Tiago()
        self.bridge = CvBridge()

    
    def transform_from_camera_frame_to_world_frame(self, camera_point):
        # http://wiki.ros.org/tf
        # http://docs.ros.org/en/indigo/api/tf/html/c++/classtf_1_1Transformer.html

        self.depth_points = rospy.wait_for_message('xtion/depth_registered/points', PointCloud2)

        self.transformer.waitForTransform('xtion_rgb_optical_frame', 'map', self.depth_points.header.stamp, rospy.Duration(2.0))

        intersection_point = PointStamped()
        intersection_point.header = self.depth_points.header
        intersection_point.point = Point(*camera_point)

        person_point = self.transformer.transformPoint('map', intersection_point)
        #print person_point
        tf_point = person_point.point
        intersection_point_world = np.array([tf_point.x, tf_point.y, tf_point.z])
        return intersection_point_world

    def transform_from_world_frame_to_camera_frame(self, world_point):
        # http://wiki.ros.org/tf
        # http://docs.ros.org/en/indigo/api/tf/html/c++/classtf_1_1Transformer.html

        self.map_points = rospy.wait_for_message('/map', OccupancyGrid)

        self.transformer.waitForTransform('map', 'xtion_rgb_optical_frame', self.map_points.header.stamp, rospy.Duration(2.0))

        intersection_point_world = PointStamped()
        intersection_point_world.header = self.map_points.header
        intersection_point_world.point = Point(*world_point)

        person_point = self.transformer.transformPoint('xtion_rgb_optical_frame', intersection_point_world)
        #print person_point
        tf_point = person_point.point
        camera_point_3d = np.array([tf_point.x, tf_point.y, tf_point.z])
        return camera_point_3d
    
    def project_depth_array_to_2d_image_pixels(self, point_3d):
        rospy.loginfo('projecting depth array to 2d image pixels')
        camera_info = rospy.wait_for_message('/xtion/rgb/camera_info', CameraInfo)
        depth_array = np.array([point_3d[0], point_3d[1], point_3d[2], 1])
        uvw = np.dot(np.array(camera_info.P).reshape((3, 4)), depth_array.transpose()).transpose()
        x = int(uvw[0] / uvw[2])
        y = int(uvw[1] / uvw[2])

        return x,y
    
    def draw_bounding_box_around_intersection_point(self, intersection_point_world):
        
        print intersection_point_world
        camera_point_3d = self.transform_from_world_frame_to_camera_frame(intersection_point_world)
        print camera_point_3d
        camera_point_2d = np.array(self.project_depth_array_to_2d_image_pixels(camera_point_3d))
        print camera_point_2d

        image_raw = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        try:
            frame = self.bridge.imgmsg_to_cv2(image_raw, 'bgr8')
            # cv2.imshow('frame', frame)
        except CvBridgeError as ex:
            rospy.logwarn(ex)
            return
        box_start_point = (camera_point_2d[0]-100),(camera_point_2d[1]-100)
        box_end_point = camera_point_2d[0]+100,camera_point_2d[1]+100
        cv2.rectangle(frame, box_start_point, box_end_point, (0,0,255), 1)
        # Plots all figures on top of an opencv image of openpose keypoints
        cv2.imshow("Bounding Box For Pointed Objects", frame)
        cv2.waitKey(5000)


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
        else:
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
        intersection_point_2d = rospy.get_param("/intersection_point_2d")
        intersection_point_3d = rospy.get_param("/intersection_point_3d")
        # print intersection_point_2d
        # print intersection_point_3d
        intersection_point_world = self.transform_from_camera_frame_to_world_frame(intersection_point_3d)

        table = self.find_table_id(intersection_point_world)
        self.approach_table(table, wait)

        self.draw_bounding_box_around_intersection_point(intersection_point_world)
        
        return 'outcome1'
