#!/usr/bin/env python

import rospy
from smach import State

import cv2
import numpy as np

# Imported for features of human robot interaction such as text to speech
from utilities import Tiago


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
        self.attributes = ['type', 'texture', 'colour', 'size', 'shape']


    def compare_current_object_with_chosen_attribute(self, current_object, attribute, current_attribute_from_user):
        
        ## MAKE TIAGO ASK THE ATTRIBUTE HERE LATER
        #current_attribute_from_user = self.dummy_attributes_from_user.get(attribute)
        current_attribute_from_feature = current_object.get(attribute)
        
        if current_attribute_from_user == current_attribute_from_feature:
            match = 1
            print attribute + " attribute matches"
        else:
            match = 0
            # self.eliminated_objects.append(current_object.get('name'))
            print attribute + " attribute does not match"
        
        return match
    
    
    def compare_current_object_if_found_within_bounding_box(self, attribute, current_object, compared_objects, current_attribute_from_user):

        for object_id in range(len(self.objects_inside_bounding_box)):
            if self.objects_inside_bounding_box[object_id].get('name') == current_object.get('name'):
                print "Current object being compared: " + current_object.get('name')
                compared_objects.append(current_object)

                match = self.compare_current_object_with_chosen_attribute(current_object, attribute, current_attribute_from_user)

                self.total_matches = np.array(np.append(self.total_matches, [match], axis = 0))
                    
            else:
                continue

        return compared_objects

    def compare_all_objects_with_chosen_attribute(self, objects_with_attributes, attribute):
        ## LOOP FOR EACH OBJECT FIRST AND THEN FOR EACH FEATURE!

        # Will be used to store all the objects compared for the current feature
        compared_objects = []
        self.total_matches = np.array([])

        current_attribute_from_user = self.gather_user_response(attribute)

        for index in range(0, len(objects_with_attributes)):
            current_object = objects_with_attributes[index]
            compared_objects = self.compare_current_object_if_found_within_bounding_box(attribute, current_object, compared_objects, current_attribute_from_user)
        
        
        if not all([ v == 0 for v in self.total_matches]):
            index = 0
            for match in self.total_matches:
                if match == 0:
                    self.eliminated_objects.append(compared_objects[index].get('name'))
                index +=1
        
        try:
            indices_for_attribute_match = np.argwhere(self.total_matches == np.amax(self.total_matches))
            # This is done to rmove extra brackets around each element of the list
            indices_for_attribute_match = [val for sublist in indices_for_attribute_match for val in sublist]
        except ValueError:  #raised if `self.total_matches` is empty.
            pass
        # print indices_for_attribute_match

        return indices_for_attribute_match, compared_objects

    def disambiguate_until_unique_feature_found(self):

        objects_with_attributes = self.tiago.object_attributes

        for attribute in self.attributes:

            indices_for_attribute_match, compared_objects= self.compare_all_objects_with_chosen_attribute(objects_with_attributes, attribute)

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

                break
                
            else:
                # LIST COMPREHENSION
                objects_with_attributes = [compared_objects[index] for index in indices_for_attribute_match]
    
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

        self.tiago.talk("ah, the object is " + user_response['transcription'])

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
        self.dummy_attributes_from_user = {
                'colour':  'yellow',
                'type':    'fresh',
                'texture': 'smooth',
                'size':    'long',
                'shape':   'curved'
            }
        

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

