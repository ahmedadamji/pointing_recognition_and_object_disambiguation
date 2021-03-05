#!/usr/bin/env python

import rospy
from smach import State

import cv2
import numpy as np

# Imported for features of human robot interaction such as text to speech
from utilities import Tiago

import math


class ObjectDisambiguation(State):
    def __init__(self):
        rospy.loginfo('ObjectDisambiguation state initialized')
        State.__init__(self, outcomes=['outcome1','outcome2'])

        #creates an instance of tiago class to use featues such as extract attributes of objects from yaml file and interact with the user
        self.tiago = Tiago()

        # Stores details of objects that have been eliminated during disambiguation
        self.eliminated_objects = []
        # Stores all matches from current attribute check, used to decide eliminated objects as well as which objects go to the next stage
        self.total_matches = np.array([])

        # Stores the types of attributes in order of use for disambiguation
        self.attributes = ['type', 'texture', 'colour', 'size', 'shape', 'position']
        # Stores the directions in terms of compass coordinates to use as part of disambiguating objects
        self.compass_directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW", "N"]

    def calculate_compass_direction_between_two_points(self, current_object_centre_point, centre_point_of_bounding_box):
        ## REFERENCE: https://www.analytics-link.com/post/2018/08/21/calculating-the-compass-direction-between-two-points-in-python

        deltaX = current_object_centre_point[0] - centre_point_of_bounding_box[0]
        deltaY = current_object_centre_point[1] - centre_point_of_bounding_box[1]
        angle_between_points = math.atan2(deltaX, deltaY)/math.pi*180
        
        if angle_between_points < 0:
            angle_between_points = 360 + angle_between_points

        else:
            angle_between_points = angle_between_points
        
        direction_index = round(angle_between_points / 45)
        compass_direction = self.compass_directions[direction_index]

        return compass_direction
    
    def get_compass_direction(self, current_object):
        # Gets the centre point of the current object on the opencv image
        if attribute == "position":
            xywh = current_object.get("xywh")
            x = xywh[0]
            y = xywh[1]
            w = xywh[2]
            h = xywh[3]
            current_object_centre_point = ((x + (w/2)),(y + (h/2)))
        
        # Gets the pointing centre point, also the centre point in bounding box, to use as reference for directions
        centre_point_of_bounding_box = rospy.get_param("/camera_point_after_object_detection_2d")

        compass_direction = self.calculate_compass_direction_between_two_points(current_object_centre_point, centre_point_of_bounding_box)

    def get_attribute_of_current_object(self, attribute, current_object):
        #current_attribute_from_user = self.dummy_attributes_from_user.get(attribute)
        if not(attribute == 'position'):
            attribute_of_current_object = current_object_attributes.get(attribute)
        
        else:
            attribute_of_current_object = self.get_compass_direction(current_object)


    def compare_current_object_with_chosen_attribute(self, current_object, current_object_attributes, attribute, current_attribute_from_user):

        if current_attribute_from_user == self.get_attribute_of_current_object(attribute, current_object):
            match = 1
            print attribute + " attribute matches"
        else:
            match = 0
            # self.eliminated_objects.append(current_object_attributes.get('name'))
            print attribute + " attribute does not match"
        
        return match
    
    
    def compare_current_object_using_attributes_from_database(self, attribute, current_object, compared_objects, current_attribute_from_user):

        for object_id in range(len(self.objects_with_attributes)):
            if self.objects_with_attributes[object_id].get('name') == current_object.get('name'):
                print "Current object being compared: " + self.objects_with_attributes[object_id].get('name')
                compared_objects.append(current_object)

                match = self.compare_current_object_with_chosen_attribute(current_object, self.objects_with_attributes[object_id], attribute, current_attribute_from_user)

                self.total_matches = np.array(np.append(self.total_matches, [match], axis = 0))
                    
            else:
                continue

        return compared_objects

    def update_eliminated_objects(self):
        # This is done so that eliminated objects are only updated if they there is at least one match for the current atrribute
        if not all([ v == 0 for v in self.total_matches]):
            # Index of each object remaining in the bounding box
            index = 0
            # Stores indices of objects from within the bounding box to be eliminated
            eliminated_objects_indices = []
            for match in self.total_matches:
                if match == 0:
                    # Updating eliminated objects
                    eliminated_objects_indices.append(index)
                    
                index +=1

        eliminated_objects_indices.reverse()
        for elemination_index in eliminated_objects_indices:
            eliminated_object = self.objects_inside_bounding_box.pop(elemination_index)
            self.eliminated_objects.append(eliminated_object.get('name'))

    def compare_all_objects_with_chosen_attribute(self, attribute):
        ## LOOP FOR EACH OBJECT FIRST AND THEN FOR EACH FEATURE!

        # Will be used to store all the objects compared for the current feature
        compared_objects = []
        indices_for_attribute_match = []
        self.total_matches = np.array([])

        current_attribute_from_user = self.gather_user_response(attribute)

        for index in range(0, len(self.objects_inside_bounding_box)):
            current_object = self.objects_inside_bounding_box[index]
            compared_objects = self.compare_current_object_using_attributes_from_database(attribute, current_object, compared_objects, current_attribute_from_user)

        # Updating eliminated objects if attributes do not match:
        self.update_eliminated_objects()
        

        ## Updating the indices of objects in bounding box where a match is found:
        indices_for_attribute_match = np.argwhere(self.total_matches == np.amax(self.total_matches))
        # This is done to rmove extra brackets around each element of the list
        indices_for_attribute_match = [val for sublist in indices_for_attribute_match for val in sublist]
        # print indices_for_attribute_match
        
        return indices_for_attribute_match, compared_objects

    def disambiguate_until_unique_feature_found(self):

        self.objects_with_attributes = self.tiago.object_attributes

        for attribute in self.attributes:

            indices_for_attribute_match, compared_objects= self.compare_all_objects_with_chosen_attribute(attribute)

            # The == condition ensures that even if it doesnt match for all objects it still goes to the next questionst
            if len(indices_for_attribute_match) == 1:
                # Two indices needed as there is a bracket around every number:
                identified_object = compared_objects[indices_for_attribute_match[0]].get('name')
                print identified_object
                self.tiago.talk("The identified object is a " + identified_object)
                print 'The eliminated objects are: '
                print self.eliminated_objects
                self.tiago.talk("The eliminated objects, in order of elimination, are: ")
                for objects in self.eliminated_objects:
                    self.tiago.talk(objects)

                return
        # Code run if diambiguation couldn't find a unique object to suit the descriptions
        self.tiago.talk("Sorry but I couldn't disambiguate the object for you, given the provided descriptions")
                
            # else:
            #     # LIST COMPREHENSION
            #     self.objects_with_attributes = [compared_objects[index] for index in indices_for_attribute_match]
    
    def gather_user_response(self, feature):

        # speech_client = actionlib.SimpleActionClient('receptionist', informationAction)
        # speech_client.wait_for_server()

        self.tiago.talk("could you please tell me the " + feature + " of the object you are pointing at?" )

        # talk('may i please get your name?', wait=True)
        # goal = informationGoal('name', 'receptionist_name')
        # tries = 0

        # while tries < 3:
        #     speech_client.send_goal(goal)
        #     speech_client.wait_for_result()
        #     result_dict['name'] = speech_client.get_result().data

        #     if not result_dict['name'] == '':
        #         break
            
        #     talk('sorry, I didn\'t catch that, may i please get your name?', wait=True)
        #     tries += 1
        
        #user_response = "yellow"
        user_response = {
            "success": True,
            "error": None,
            "transcription": None
        }

        text = raw_input('Please type your response : ')
        user_response['transcription'] = text
        # Reinforces to the user, the attribute collected
        self.tiago.talk("ah, the " + feature + " of the object is " + user_response['transcription'])

        # goal = informationGoal('drink', 'receptionist_drink')
        # tries = 0

        return user_response['transcription']

        # while tries < 3:
        #     speech_client.send_goal(goal)
        #     speech_client.wait_for_result()
        #     result_dict['drink'] = speech_client.get_result().data

        #     if not result_dict['drink'] == '':
        #         break
            
        #     talk('sorry, I didn\'t catch that, may i please get your favourite drink?', wait=True)
        #     tries += 1

        # talk("and your favourite drink is " + result_dict['drink'], wait=True)



    def execute(self, userdata):
        rospy.loginfo('ObjectDisambiguation state executing')

        self.tiago.talk("My name is Ahmed and I am the robo maker")

        self.objects_inside_bounding_box = rospy.get_param('/objects_inside_bounding_box')

        # print objects_inside_bounding_box
        # print objects_inside_bounding_box[0].get('name')

        self.attributes_from_user = []
        # self.dummy_attributes_from_user = {
        #         'colour':  'yellow',
        #         'type':    'fresh',
        #         'texture': 'smooth',
        #         'size':    'long',
        #         'shape':   'curved'
        #     }
        

        self.disambiguate_until_unique_feature_found()
                



        # for object in len(list_of_objects_within_bounding_box):
        #     attributes_from_detected_objects = list_of_objects_within_bounding_box[object].attributes
        #     print "Current object being compared: " + list_of_objects_within_bounding_box[object].name
        #     for attribute in len(attributes_from_user)
        #         if attributes_from_user[attribute] == attributes_from_detected_objects[attribute]:
        #             match += 1
        #         else:
        #             print "Attribute does not match"
        #     self.total_matches.append(match)
        # indices_for_attribute_match = np.argwhere(self.total_matches == np.amax(self.total_matches))
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

