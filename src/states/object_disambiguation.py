#!/usr/bin/env python

import rospy
import actionlib
from smach import State

import cv2
import numpy as np
import math

# Imported for features of human robot interaction such as text to speech
from utilities import Tiago, Util



class ObjectDisambiguation(State):
    def __init__(self):
        rospy.loginfo('ObjectDisambiguation state initialized')
        State.__init__(self, outcomes=['outcome1','outcome2'])

        #creates an instance of tiago class to interact with the user
        self.tiago = Tiago()
        #creates an instance of util class to use featues such as extract attributes of objects from yaml file and transform point frames
        self.util = Util()

        # Stores details of objects that have been eliminated during disambiguation
        self.eliminated_objects = []
        # Stores details of objects that are not part of disambiguation but were found inside the bounding box
        self.objects_inside_bounding_box_not_compared = []
        # Stores all matches from current attribute check, used to decide eliminated objects as well as which objects go to the next stage
        self.total_matches = np.array([])

        # Stores the types of attributes in order of use for disambiguation
        self.attributes = 'position', 'colour', 'texture', 'type', 'size', 'shape'
        # Stores the possible directions in terms of compass coordinates to use as part of disambiguating objects
        self.compass_directions = ["north", "east", "south", "west", "north", "centre"]
        # Stores the possible directions to use as part of disambiguating objects
        self.standard_directions = ["up", "right", "down", "left", "up", "centre"]

        self.objects_with_attributes = self.util.object_attributes
        self.list_of_attributes = self.util.list_of_attributes
        self.list_of_objects_capable_of_disambiguation = self.util.list_of_objects_capable_of_disambiguation

    def convert_standard_directions_to_compass_directions(self, direction_of_current_object):
        #Converting north, south, east, west TO up, down , right , left
        if direction_of_current_object.lower() in self.standard_directions:
            index = self.standard_directions.index(direction_of_current_object.lower())
            return self.compass_directions[index]
        else:
            return direction_of_current_object


    def calculate_compass_direction_between_two_points(self, point_of_interest, reference_point = [0,0]):
        ## REFERENCE: https://www.analytics-link.com/post/2018/08/21/calculating-the-compass-direction-between-two-points-in-python

        # point_of_interest = self.util.convert_from_image_to_cartesian_coordinate_system(point_of_interest)
        # reference_point = self.util.convert_from_image_to_cartesian_coordinate_system(reference_point)
        deltaX = point_of_interest[0] - reference_point[0]
        deltaY = point_of_interest[1] - reference_point[1]
        angle_between_points = math.atan2(deltaX, deltaY)/math.pi*180
        distance_between_points = math.hypot(deltaX, deltaY)

        if distance_between_points < 0.02:
            compass_direction = self.compass_directions[5]
            return compass_direction

        
        elif angle_between_points < 0:
            angle_between_points = 360 + angle_between_points

        else:
            angle_between_points = angle_between_points
        
        direction_index = int(round(angle_between_points / 90))
        compass_direction = self.compass_directions[direction_index]

        return compass_direction

    def transfer_coordinate_wrt_person_and_reference_object(self, point_of_interest, reference_point, view_point):
        #http://motion.cs.illinois.edu/RoboticSystems/CoordinateTransformations.html
        # section 2.10
        # Or check another reference to transform frames in robotics
        # Pass in a genuine reference_point in world frame
        reference_point = [-10,-8.9]

        #Get distance between two points:
        reference_vector = np.array([reference_point[0],reference_point[1]])
        view_point_vector = np.array([view_point[0],view_point[1]])
        new_y_axis_vector = np.subtract(reference_vector,view_point_vector)
        magnitude_new_y_axis_vector = np.linalg.norm(new_y_axis_vector)

        # To find the angle to rotate the coordinate system by:
        # Here I had to concider the value change of angle when it lies in diffrent quadrants:
        theta = math.acos((reference_point[0]-view_point[0])/magnitude_new_y_axis_vector)

        if view_point[1] > reference_point[1]:
            theta = 2*(math.pi) - theta
        
        #theta_a_b  # - (pi/2) because the y axis of the new coordinate frame is aligned with the rotation vector and therefore the x axis will be rotated by 90 degrees less than this.
        rotation_angle = theta-((math.pi)/2.0)
       
        wxr = reference_point[0]
        wyr = reference_point[1]
        # Finding the transformation matrix of the world frame to the reference frame
        wtr = [[math.cos(rotation_angle), -math.sin(rotation_angle), wxr], 
               [math.sin(rotation_angle), math.cos(rotation_angle), wyr],
               [0, 0, 1]]
        
        wtr = np.array(wtr)

        # Finding the transformation matrix of the reference frame to the world frame
        rtw = np.linalg.inv(wtr)

        wp = [point_of_interest[0],point_of_interest[1],1]
        wp = np.array(wp).reshape(3,1)

        rp = np.matmul(rtw, wp)
        point_of_interest_transformed = [rp[0],rp[1]]
        print point_of_interest_transformed


        return point_of_interest_transformed
    
    def get_compass_direction(self, current_object):

        # Gets the centre point of the current object on the opencv image
        # xywh = current_object.get("xywh")
        # x = xywh[0]
        # y = xywh[1]
        # w = xywh[2]
        # h = xywh[3]
        # current_object_centre_point = ((x + (w/2)),(y + (h/2)))
        
        # Gets the World coordinate of the object:
        current_object_world_coordinate = current_object.get("world_coordinate")

        # Gets the World Coordinates of person's head:
        self.person_head_world_coordinate

        #USING REFERENCE OBJECT AS CURRENT OBJECT AS WELL FOR NOW, CHANGE TO SPECIFIC REFERENCE OBJECT LATER
        current_object_world_coordinate_transformed = self.transfer_coordinate_wrt_person_and_reference_object(current_object_world_coordinate, current_object_world_coordinate, self.person_head_world_coordinate)        

        # Gets the pointing centre point, also the centre point in bounding box, to use as reference for directions
        # centre_point_of_bounding_box = rospy.get_param("/camera_point_after_object_detection_2d")

        compass_direction = self.calculate_compass_direction_between_two_points(current_object_world_coordinate_transformed)
        return compass_direction

    def get_attribute_of_current_object(self, attribute, current_object, current_object_attributes):
        #current_attribute_from_user = self.dummy_attributes_from_user.get(attribute)
        # Can add here more conditions for example if attribute needs to be extracted by means of opencv / other means in the future.
        if not(attribute == 'position'):
            attribute_of_current_object = current_object_attributes.get(attribute)
        elif (attribute == 'position'):
            compass_direction = self.get_compass_direction(current_object)
            attribute_of_current_object = compass_direction

            # This block of code if just for printing the direction found of the objects in the bounding box and are not essential to the functionality
            index = self.compass_directions.index(compass_direction)
            standard_direction = self.standard_directions[index]
            if compass_direction is not 'centre':
                print (current_object.get('name') + " was found in the: " + compass_direction + " / " + standard_direction)
            else:
                print (current_object.get('name') + " was found in the: " + compass_direction)
            
        

        return attribute_of_current_object
        


    def compare_current_object_with_chosen_attribute(self, current_object, current_object_attributes, attribute, current_attribute_from_user):

        if current_attribute_from_user.lower() == self.get_attribute_of_current_object(attribute, current_object, current_object_attributes).lower():
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
        
        # Stores indices of objects from within the bounding box to be eliminated
        eliminated_objects_indices = []
        if not all([ v == 0 for v in self.total_matches]):
            # Index of each object remaining in the bounding box
            index = 0
            for match in self.total_matches:
                if match == 0:
                    # Updating eliminated objects
                    eliminated_objects_indices.append(index)
                    
                index +=1

        eliminated_objects_indices.reverse()
        for elemination_index in eliminated_objects_indices:
            eliminated_object = self.objects_inside_bounding_box.pop(elemination_index)
            self.eliminated_objects.append(eliminated_object.get('name'))

        
    def gather_user_response(self, attribute):

        self.tiago.talk("could you please tell me the " + attribute + " of the object you are pointing at?" )
        
        # Stores a list of valid responses
        valid_responses = self.list_of_attributes.get(attribute)

        user_response = self.tiago.get_data_from_user("text", valid_responses, attribute) # request_type, valid_responses, type_of_data
        return user_response


    def compare_all_objects_with_chosen_attribute(self, attribute):
        ## LOOP FOR EACH OBJECT FIRST AND THEN FOR EACH FEATURE!

        # Will be used to store all the objects compared for the current feature
        compared_objects = []
        indices_for_attribute_match = []
        self.total_matches = np.array([])

        current_attribute_from_user = self.gather_user_response(attribute)

        index = 0
        while index < len(self.objects_inside_bounding_box):
            # IF - ELSE block to ensure only objects capable of disambiguation are used for this state
            if self.objects_inside_bounding_box[index].get('name') in self.list_of_objects_capable_of_disambiguation:
                current_object = self.objects_inside_bounding_box[index]
                compared_objects = self.compare_current_object_using_attributes_from_database(attribute, current_object, compared_objects, current_attribute_from_user)
                index +=1
            else:
                self.objects_inside_bounding_box_not_compared.append(self.objects_inside_bounding_box.pop(index))
            

        # Updating eliminated objects if attributes do not match:
        self.update_eliminated_objects()
        

        ## Updating the indices of objects in bounding box where a match is found:
        indices_for_attribute_match = np.argwhere(self.total_matches == np.amax(self.total_matches))
        # This is done to rmove extra brackets around each element of the list
        indices_for_attribute_match = [val for sublist in indices_for_attribute_match for val in sublist]
        # print indices_for_attribute_match
        
        return indices_for_attribute_match, compared_objects
    
    def notify_of_objectes_detected_but_not_part_of_disambiguation(self):
        # Called if some objects that are not programmed to be disambiguated are found within the bounding box.
        if len(self.objects_inside_bounding_box_not_compared) is not 0:
            self.tiago.talk("The objects that were detected close to the location of pointing, but the ones I am not yet programmed to disambiguate for you are: ")
            for objects in self.objects_inside_bounding_box_not_compared:
                self.tiago.talk(objects.get('name'))

    def disambiguate_until_unique_feature_found(self):

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
                
                self.notify_of_objectes_detected_but_not_part_of_disambiguation()
                return

        
        # Code run if diambiguation couldn't find a unique object to suit the descriptions
        self.tiago.talk("Sorry but I couldn't disambiguate the object for you, given the provided descriptions")
        self.notify_of_objectes_detected_but_not_part_of_disambiguation()


    # PUT TEXT AND SPEECH BOTH IN TIAGO.PY


    def execute(self, userdata):
        rospy.loginfo('ObjectDisambiguation state executing')

        self.tiago.talk("My name is Ahmed and I am the robo maker")
        self.objects_inside_bounding_box = rospy.get_param('/objects_inside_bounding_box')
        self.person_head_world_coordinate = rospy.get_param('/person_head_world_coordinate')
        

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

