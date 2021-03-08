#!/usr/bin/python
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

# Importing for speech recognition, interaction and replying to person
# import speech_recognition as sr
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import pyttsx

import tf
import numpy as np


from geometry_msgs.msg import Point, PointStamped, Pose
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid



class Util:
    def __init__(self):

        self.load_object_features()
        self.transformer = tf.TransformListener()

    def transform_from_camera_frame_to_world_frame(self, camera_point_3d):
        # http://wiki.ros.org/tf
        # http://docs.ros.org/en/indigo/api/tf/html/c++/classtf_1_1Transformer.html

        self.depth_points = rospy.wait_for_message('xtion/depth_registered/points', PointCloud2)

        self.transformer.waitForTransform('xtion_rgb_optical_frame', 'map', self.depth_points.header.stamp, rospy.Duration(2.0))

        camera_point_stamped = PointStamped()
        camera_point_stamped.header = self.depth_points.header
        camera_point_stamped.point = Point(*camera_point_3d)

        person_point = self.transformer.transformPoint('map', camera_point_stamped)
        #print person_point
        tf_point = person_point.point
        world_point = np.array([tf_point.x, tf_point.y, tf_point.z])
        return world_point

    def transform_from_world_frame_to_camera_frame(self, world_point):
        # http://wiki.ros.org/tf
        # http://docs.ros.org/en/indigo/api/tf/html/c++/classtf_1_1Transformer.html

        self.map_points = rospy.wait_for_message('/map', OccupancyGrid)

        self.transformer.waitForTransform('map', 'xtion_rgb_optical_frame', self.map_points.header.stamp, rospy.Duration(2.0))

        world_point_stamped = PointStamped()
        world_point_stamped.header = self.map_points.header
        world_point_stamped.point = Point(*world_point)

        person_point = self.transformer.transformPoint('xtion_rgb_optical_frame', world_point_stamped)
        #print person_point
        tf_point = person_point.point
        camera_point_3d = np.array([tf_point.x, tf_point.y, tf_point.z])
        return camera_point_3d

    def load_object_features(self):

        object_attributes_parm = rospy.get_param('/object_attributes')
        self.object_attributes = [
            {
                'name':    objects['name'],
                'colour':  objects['colour'],
                'type':    objects['type'],
                'texture': objects['texture'],
                'size':    objects['size'],
                'shape':   objects['shape']
            }
            for objects in object_attributes_parm
        ]
        
        list_of_attributes_parm = rospy.get_param('/list_of_attributes')[0]
        self.list_of_attributes = {
            'colour':  list_of_attributes_parm['colour'],
            'type':    list_of_attributes_parm['type'],
            'texture': list_of_attributes_parm['texture'],
            'size':    list_of_attributes_parm['size'],
            'shape':   list_of_attributes_parm['shape'],
            'position': list_of_attributes_parm['position']
        }

        self.list_of_objects_capable_of_disambiguation = rospy.get_param('/list_of_objects_capable_of_disambiguation')



    def shutdown(self):

        if self.play_motion_goal_sent:
            self.play_motion.cancel_goal()
            rospy.loginfo("Stop Robot")
            rospy.sleep(1)
