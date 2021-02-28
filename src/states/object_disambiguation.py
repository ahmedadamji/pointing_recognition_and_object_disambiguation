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


    def compare_chosen_attribute(self, current_attribute_from_user, current_attribute_from_feature, match):
        if current_attribute_from_user == current_attribute_from_feature:
            match += 1
            print "Attribute matches"
        else:
            print "Attribute does not match"
        
        return match
    
    def get_attribute_matches_with_current_object(self, current_object, attribute):

        match = 0
        ## MAKE TIAGO ASK THE ATTRIBUTE HERE LATER
        current_attribute_from_user = self.dummy_attributes_from_user.get(attribute)
        current_attribute_from_feature = current_object.get(attribute)
        match = self.compare_chosen_attribute(current_attribute_from_user, current_attribute_from_feature, match)

        # current_attribute_from_user = self.dummy_attributes_from_user.get('colour')
        # current_attribute_from_feature = current_object.get('colour')
        # match = self.compare_chosen_attribute(current_attribute_from_user, current_attribute_from_feature, match)

        # current_attribute_from_user = self.dummy_attributes_from_user.get('type')
        # current_attribute_from_feature = current_object.get('type')
        # match = self.compare_chosen_attribute(current_attribute_from_user, current_attribute_from_feature, match)

        # current_attribute_from_user = self.dummy_attributes_from_user.get('texture')
        # current_attribute_from_feature = current_object.get('texture')
        # match = self.compare_chosen_attribute(current_attribute_from_user, current_attribute_from_feature, match)

        # current_attribute_from_user = self.dummy_attributes_from_user.get('size')
        # current_attribute_from_feature = current_object.get('size')
        # match = self.compare_chosen_attribute(current_attribute_from_user, current_attribute_from_feature, match)

        # current_attribute_from_user = self.dummy_attributes_from_user.get('shape')
        # current_attribute_from_feature = current_object.get('shape')
        # match = self.compare_chosen_attribute(current_attribute_from_user, current_attribute_from_feature, match)

        return match
    
    def get_matches_for_all_objects_in_bounding_box(self, attribute, current_object, compared_objects, total_matches):
        for object_id in range(len(self.objects_inside_bounding_box)):
            if self.objects_inside_bounding_box[object_id].get('name') == current_object.get('name'):
                print "Current object being compared: " + current_object.get('name')
                compared_objects.append(current_object)

                match = self.get_attribute_matches_with_current_object(current_object, attribute)
                if match == 0:
                    self.eliminated_objects_indices.append(object_id)

                total_matches = np.array(np.append(total_matches, [match], axis = 0))
            else:
                continue

        return compared_objects, total_matches

    def execute(self, userdata):
        rospy.loginfo('ObjectDisambiguation state executing')

        self.tiago.speak("My name is Ahmed and I am the robo maker")

        self.objects_inside_bounding_box = rospy.get_param('/objects_inside_bounding_box')
        self.eliminated_objects = []
        self.eliminated_objects_indices = []
        # print objects_inside_bounding_box
        # print objects_inside_bounding_box[0].get('name')
        attributes_from_user = []
        self.dummy_attributes_from_user = {
                'colour':  'yellow',
                'type':    'fresh',
                'texture': 'smooth',
                'size':    'long',
                'shape':   'curved'
            }
        total_matches = np.array([])
        compared_objects = []
        detection_confidence = []

        object_attributes = self.tiago.object_attributes
        ## LOOP FOR EACH OBJECT FIRST AND THEN FOR EACH FEATURE!
        for index in range(0, len(object_attributes)):
            current_object = object_attributes[index]
            compared_objects, total_matches = self.get_matches_for_all_objects_in_bounding_box('colour', current_object, compared_objects, total_matches)
        # Reversing indices to pop as the indices will not change while going through the for loop
        self.eliminated_objects_indices.reverse()
        for elemination_index in range(len(self.eliminated_objects_indices)):
            self.eliminated_objects.append(self.objects_inside_bounding_box.pop(self.eliminated_objects_indices[elemination_index]))
        
        try:
            indices_for_attribute_match = np.argwhere(total_matches == np.amax(total_matches))
        except ValueError:  #raised if `total_matches` is empty.
            pass
        print indices_for_attribute_match
        if len(indices_for_attribute_match) == 1:
            # Two indices needed as there is a bracket around every number:
            identified_object = compared_objects[indices_for_attribute_match[0][0]].get('name')
            print identified_object
            self.tiago.speak("The identified object is a " + identified_object)
        else:
            # LIST COMPREHENSION
            # THE 0 BECAUSE THERE IS AN EXTRA BRACKET AROUND EACH ITEM
            object_attributes = [compared_objects[index][0] for index in indices_for_attribute_match]
            ## LOOP FOR EACH OBJECT FIRST AND THEN FOR EACH FEATURE!
            for index in range(0, len(object_attributes)):
                current_object = object_attributes[index]
                compared_objects, total_matches = self.get_matches_for_all_objects_in_bounding_box('colour', current_object, compared_objects, total_matches)
            
            try:
                indices_for_attribute_match = np.argwhere(total_matches == np.amax(total_matches))
            except ValueError:  #raised if `total_matches` is empty.
                pass
            print indices_for_attribute_match
            if len(indices_for_attribute_match) == 1:
                # Two indices needed as there is a bracket around every number:
                identified_object = compared_objects[indices_for_attribute_match[0][0]].get('name')
                print identified_object
                self.tiago.speak("The identified object is a " + identified_object)







        # else:
        #     ## LOOP FOR EACH OBJECT FIRST AND THEN FOR EACH FEATURE!
        #     total_matches = np.array([])
        #     compared_objects = []
        #     for index in range(len(indices_for_attribute_match)):
        #         current_object = compared_objects[indices_for_attribute_match[index][0]]
        #         compared_objects, total_matches = self.get_matches_for_all_objects_in_bounding_box('type', current_object, compared_objects, total_matches)
        #     try:
        #         indices_for_attribute_match = np.argwhere(total_matches == np.amax(total_matches))
        #     except ValueError:  #raised if `total_matches` is empty.
        #         pass
        #     print indices_for_attribute_match
        #     if len(indices_for_attribute_match) == 1:
        #         # Two indices needed as there is a bracket around every number:
        #         identified_object = compared_objects[indices_for_attribute_match[0][0]].get('name')
        #         print identified_object
        #         self.tiago.speak("The identified object is a " + identified_object)






        # for object in len(list_of_objects_within_bounding_box):
        #     attributes_from_detected_objects = list_of_objects_within_bounding_box[object].attributes
        #     print "Current object being compared: " + list_of_objects_within_bounding_box[object].name
        #     for attribute in len(attributes_from_user)
        #         if attributes_from_user[attribute] == attributes_from_detected_objects[attribute]:
        #             match += 1
        #         else:
        #             print "Attribute does not match"
        #     total_matches.append(match)
        # indices_for_attribute_match = np.argwhere(total_matches == np.amax(total_matches))
        # if len(indices_for_attribute_match) == 1:
        #     identified_object = list_of_objects_within_bounding_box[indices_for_attribute_match[0]]
        #     return identified_object
        # else:
        #     # Return Object With Highest Confidence
        #     for object in len(indices_for_attribute_match):
        #         detection_confidence.append(list_of_objects_within_bounding_box[indices_for_attribute_match[object]].confidence)
        #     highest_confidence_index = indices_for_attribute_match[np.argmax(detection_confidence)]
        #     identified_object = list_of_objects_within_bounding_box[highest_confidence_index]
        #     return identified_object








        # To destroy cv2 window at the end of state
        cv2.destroyAllWindows()

        return 'outcome1'
