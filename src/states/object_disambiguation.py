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


    def compare_current_attributes(self, current_attribute_from_user, current_attribute_from_feature, match):
        if current_attribute_from_user == current_attribute_from_feature:
            match += 1
            print "Attribute matches"
        else:
            print "Attribute does not match"
        
        return match
    
    def compare_all_attributes(self, dummy_attributes_from_user, current_object, attribute):

        match = 0
        ## MAKE TIAGO ASK THE ATTRIBUTE HERE LATER
        current_attribute_from_user = dummy_attributes_from_user.get(attribute)
        current_attribute_from_feature = current_object.get(attribute)
        match = self.compare_current_attributes(current_attribute_from_user, current_attribute_from_feature, match)

        # current_attribute_from_user = dummy_attributes_from_user.get('colour')
        # current_attribute_from_feature = current_object.get('colour')
        # match = self.compare_current_attributes(current_attribute_from_user, current_attribute_from_feature, match)

        # current_attribute_from_user = dummy_attributes_from_user.get('type')
        # current_attribute_from_feature = current_object.get('type')
        # match = self.compare_current_attributes(current_attribute_from_user, current_attribute_from_feature, match)

        # current_attribute_from_user = dummy_attributes_from_user.get('texture')
        # current_attribute_from_feature = current_object.get('texture')
        # match = self.compare_current_attributes(current_attribute_from_user, current_attribute_from_feature, match)

        # current_attribute_from_user = dummy_attributes_from_user.get('size')
        # current_attribute_from_feature = current_object.get('size')
        # match = self.compare_current_attributes(current_attribute_from_user, current_attribute_from_feature, match)

        # current_attribute_from_user = dummy_attributes_from_user.get('shape')
        # current_attribute_from_feature = current_object.get('shape')
        # match = self.compare_current_attributes(current_attribute_from_user, current_attribute_from_feature, match)

        return match
    
    def find_total_matches_for_objects_by_attribute(self, attribute, current_object, dummy_attributes_from_user, compared_objects, total_matches):
        for object_id in range(len(self.objects_inside_bounding_box)):
            if self.objects_inside_bounding_box[object_id].get('name') == current_object.get('name'):
                print "Current object being compared: " + current_object.get('name')
                compared_objects.append(current_object)

                match = self.compare_all_attributes(dummy_attributes_from_user, current_object, attribute)

                total_matches = np.array(np.append(total_matches, [match], axis = 0))
            else:
                continue
        return compared_objects, total_matches

    def execute(self, userdata):
        rospy.loginfo('ObjectDisambiguation state executing')

        self.tiago.speak("My name is Ahmed and I am the robo maker")

        self.objects_inside_bounding_box = rospy.get_param('/objects_inside_bounding_box')
        # print objects_inside_bounding_box
        # print objects_inside_bounding_box[0].get('name')
        attributes_from_user = []
        dummy_attributes_from_user = {
                'colour':  'yellow',
                'type':    'fresh',
                'texture': 'smooth',
                'size':    'long',
                'shape':   'curved'
            }
        total_matches = np.array([])
        detection_confidence = []
        compared_objects = []


        ## LOOP FOR EACH OBJECT FIRST AND THEN FOR EACH FEATURE!
        for index in range(0, len(self.tiago.object_attributes)):
            current_object = self.tiago.object_attributes[index]
            compared_objects, total_matches = self.find_total_matches_for_objects_by_attribute('colour', current_object, dummy_attributes_from_user, compared_objects, total_matches)
        
        try:
            max_attribute_matches = np.argwhere(total_matches == np.amax(total_matches))
        except ValueError:  #raised if `total_matches` is empty.
            pass
        print max_attribute_matches
        if len(max_attribute_matches) == 1:
            # Two indices needed as there is a bracket around every number:
            identified_object = compared_objects[max_attribute_matches[0][0]].get('name')
            print identified_object
            self.tiago.speak("The identified object is a " + identified_object)
        else:
            ## LOOP FOR EACH OBJECT FIRST AND THEN FOR EACH FEATURE!
            for index in range(len(max_attribute_matches)):
                current_object = compared_objects[max_attribute_matches[index][0]]
                compared_objects, total_matches = self.find_total_matches_for_objects_by_attribute('type', current_object, dummy_attributes_from_user, compared_objects, total_matches)
            try:
                max_attribute_matches = np.argwhere(total_matches == np.amax(total_matches))
            except ValueError:  #raised if `total_matches` is empty.
                pass
            print max_attribute_matches
            if len(max_attribute_matches) == 1:
                # Two indices needed as there is a bracket around every number:
                identified_object = compared_objects[max_attribute_matches[0][0]].get('name')
                print identified_object
                self.tiago.speak("The identified object is a " + identified_object)






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
