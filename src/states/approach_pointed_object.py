#!/usr/bin/env python
import rospy
import actionlib
import tf
import numpy as np

from smach import State

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

    
    # def transform_from_camera_frame_to_world_frame(camera_point):
    #     self.cam_model.fromCameraInfo(self.info_msg_left,self.info_msg_right)
    #     point_msg.pose.position.x= camera_point[0]
    #     point_msg.pose.position.y=camera_point[1]
    #     point_msg.pose.position.z= camera_point[2]
    #     point_msg.pose.orientation.x=0
    #     point_msg.pose.orientation.y=0
    #     point_msg.pose.orientation.z=0
    #     point_msg.pose.orientation.w=1
    #     point_msg.header.stamp = rospy.Time.now()
    #     point_msg.header.frame_id = self.cam_model.tfFrame()
    #     self.listener.waitForTransform(self.cam_model.tfFrame(), "world", rospy.Time.now(), rospy.Duration(1.0))
    #     tf_point = self.listener.transformPose("world", point_msg)

    def execute(self, userdata, wait=True):

        # intersection_point = rospy.wait_for_message('/intersection_point', IntersectionData)
        # print intersection_point.intersection_point_2d
        # print intersection_point.intersection_point_3d
        intersection_point_2d = rospy.get_param("/intersection_point_2d")
        intersection_point_3d = rospy.get_param("/intersection_point_3d")
        print intersection_point_2d
        print intersection_point_3d



        depth_points = rospy.wait_for_message('xtion/depth_registered/points', PointCloud2)

        self.transformer.waitForTransform('xtion_rgb_optical_frame', 'map', depth_points.header.stamp, rospy.Duration(2.0))

        intersection_point = PointStamped()
        intersection_point.header = depth_points.header
        intersection_point.point = Point(*intersection_point_3d)

        person_point = self.transformer.transformPoint('map', intersection_point)
        #print person_point
        tf_point = person_point.point
        intersection_point_world = np.array([tf_point.x, tf_point.y, tf_point.z])
        print intersection_point_world

        cv2.waitKey(0)
        
        return 'outcome1'
