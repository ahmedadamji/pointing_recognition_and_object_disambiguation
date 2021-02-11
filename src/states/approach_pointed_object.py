#!/usr/bin/env python
import rospy
import actionlib

from smach import State
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
from pointing_recognition.msg import IntersectionData


import cv2


class ApproachPointedObject(State):
    def __init__(self):
        rospy.loginfo('ApproachPointedObject state initialized')
        
        State.__init__(self, outcomes=['outcome1','outcome2'])

    
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








        cv2.waitKey(0)
        
        return 'outcome1'
