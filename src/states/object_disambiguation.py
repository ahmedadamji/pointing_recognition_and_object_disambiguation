#!/usr/bin/env python
import rospy
import actionlib
from smach import State
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from sensor_msgs.srv import SetCameraInfo
import sensor_msgs.point_cloud2 as pc2

from pointing_recognition.msg import IntersectionData

from geometry_msgs.msg import PoseStamped

import cv2
import math
import numpy as np

# Imported for features of human robot interaction such as text to speech
from utilities import Tiago


class ObjectDisambiguation(State):
    def __init__(self):
        rospy.loginfo('ObjectDisambiguation state initialized')
        State.__init__(self, outcomes=['outcome1','outcome2'])
        self.tiago = Tiago()


    

    def execute(self, userdata):

        self.tiago.talk("My name is Ahmed and I am the robo maker")

        # rospy.loginfo('ObjectDisambiguation state executing')
        # list_of_objects_within_bounding_box = []
        # attributes_from_user = []
        # total_matches_for_all_objects = []
        # detection_confidence = []
        # for object in len(list_of_objects_within_bounding_box):
        #     attributes_from_detected_objects = list_of_objects_within_bounding_box[object].attributes
        #     print "Current object being compared: " + list_of_objects_within_bounding_box[object].name
        #     for attribute in len(attributes_from_user)
        #         if attributes_from_user[attribute] == attributes_from_detected_objects[attribute]:
        #             match += 1
        #         else:
        #             print "Attribute does not match"
        #     total_matches.append(match)
        # max_attribute_matches = np.argwhere(total_matches == np.amax(total_matches))
        # if len(max_attribute_matches) == 1:
        #     identified_object = list_of_objects_within_bounding_box[max_attribute_matches[0]]
        #     return identified_object
        # else:
        #     # Return Object With Highest Confidence
        #     for object in len(max_attribute_matches):
        #         detection_confidence.append(list_of_objects_within_bounding_box[max_attribute_matches[object]].confidence)
        #     highest_confidence_index = max_attribute_matches[np.argmax(detection_confidence)]
        #     identified_object = list_of_objects_within_bounding_box[highest_confidence_index]
        #     return identified_object



        # To destroy cv2 window at the end of state
        cv2.destroyAllWindows()

        return 'outcome1'
