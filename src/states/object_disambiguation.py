#!/usr/bin/env python

import rospy
import actionlib
from smach import State

import cv2
import numpy as np
import math
from sympy import Point3D

from scipy.spatial import distance

from collections import Counter
from itertools import chain




class ObjectDisambiguation(State):
    def __init__(self, tiago, util):
        rospy.loginfo('ObjectDisambiguation state initialized')
        State.__init__(self, outcomes=['outcome1','outcome2'])

        #creates an instance of tiago class to interact with the user
        self.tiago = tiago
        #creates an instance of util class to use featues such as extract attributes of objects from yaml file and transform point frames
        self.util = util

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
        self.standard_directions = ["front", "right", "behind", "left", "front", "centre"]
        # Number of directions that can be referenced will be stored here, if the reference object is closest to the person, then only 2 (right or left) 
        # directions make sense, the default value is 4
        self.directions = 4

        self.objects_with_attributes = self.util.object_attributes
        self.list_of_attributes = self.util.list_of_attributes

    def convert_standard_directions_to_compass_directions(self, direction_of_current_object):
        #Converting north, south, east, west TO front, behind , right , left
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

        # Test it out if this distance of 0.02 is useful or if i should change it back to 0
        if distance_between_points < 0.02:
            compass_direction = self.compass_directions[5]
            return compass_direction

        
        elif angle_between_points < 0:
            angle_between_points = 360 + angle_between_points

        else:
            angle_between_points = angle_between_points

        if self.directions == 4:
            max_angle = 90
            direction_index = (int(round(angle_between_points / max_angle)))
        elif self.directions ==2:
            max_angle = 180
            direction_index = 2*(int(round(angle_between_points / max_angle)))
        compass_direction = self.compass_directions[direction_index]

        return compass_direction
        


    def transfer_coordinate_wrt_person_and_reference_object(self, point_of_interest, reference_point, view_point):
        #http://motion.cs.illinois.edu/RoboticSystems/CoordinateTransformations.html
        # section 2.10
        # Or check another reference to transform frames in robotics

        # Pass in a genuine reference_point in world frame
        # For now it is the detected location of pointing as reference
        # intersection_point_world = rospy.get_param("/intersection_point_world")
        # reference_point = intersection_point_world

        # # Using the centre of the current table as the reference for directions between objects from viewpoint of person
        # cuboid = self.current_table.get('cuboid')
        # cuboid_max = Point3D(cuboid['max_xyz'])
        # cuboid_min = Point3D(cuboid['min_xyz'])
        # # cuboid_midpoint = Point3D((cuboid_max.x+cuboid_min.x)/2, (cuboid_max.y+cuboid_min.y)/2, (cuboid_max.z+cuboid_min.z)/2)
        # cuboid_midpoint = cuboid_max.midpoint(cuboid_min)
        # reference_point = [cuboid_midpoint.x, cuboid_midpoint.y]


        #reference_point = self.reference_object.get('world_coordinate')

        #Get distance between two points:
        reference_vector = np.array([reference_point[0],reference_point[1]])
        view_point_vector = np.array([view_point[0],view_point[1]])
        new_y_axis_vector = np.subtract(reference_vector,view_point_vector)
        new_y_axis_vector_magnitude = np.linalg.norm(new_y_axis_vector)

        # To find the angle to rotate the coordinate system by:
        # Here I had to concider the value change of angle when it lies in diffrent quadrants:
        theta = math.acos((reference_point[0]-view_point[0])/new_y_axis_vector_magnitude)

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

    def select_reference_object(self):

        object_with_unique_feature = None
        world_coordinate_of_object_with_unique_feature = None
        if len(self.unique_features) is not 0:
            # HERE I CAN USE ANY OF THE UNIQUE FEATURES TO REFERNCE THE OBJECTS
            # using the first element only as there can be more than one unique feature found
            unique_feature = self.unique_features[0]
            # print unique_feature
            for current_object in self.objects_within_pointing_bounding_box_with_attributes:
                if unique_feature in current_object:
                    object_with_unique_feature = current_object[0]
            for current_object in self.objects_within_pointing_bounding_box:
                if current_object.get('name') == object_with_unique_feature:
                    world_coordinate_of_object_with_unique_feature = current_object.get('world_coordinate')
            reference_object = {
                "name": object_with_unique_feature,
                "unique_feature": unique_feature,
                "world_coordinate":world_coordinate_of_object_with_unique_feature
            }
            self.directions = 4

        else:
            reference_object = self.closest_object
            # only right or left directions as all of them will essentially be behind the nearest object
            self.directions = 2
        
        return reference_object
    
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

        # Finds the reference object that has a unique attribute compared to the rest and transfers the coordinate frame from the world frame
        # to a reference frame centred around this reference object and aligned with the line of sight of the person answering questions
        reference_object_world_coordinate = self.reference_object.get('world_coordinate')
        current_object_world_coordinate_transformed = self.transfer_coordinate_wrt_person_and_reference_object(current_object_world_coordinate, reference_object_world_coordinate, self.person_head_world_coordinate)        

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

            # This block of code if just for printing the direction found of the objects in the bounding box and are not essential to the functionality
            index = self.compass_directions.index(compass_direction)
            standard_direction = self.standard_directions[index]
            if compass_direction is not 'centre':
                print (current_object.get('name') + " was found in the: " + compass_direction + " / " + standard_direction)
            else:
                print (current_object.get('name') + " was found in the: " + compass_direction)
            
            attribute_of_current_object = [compass_direction, standard_direction]
            
        

        return attribute_of_current_object
        


    def compare_current_object_with_chosen_attribute(self, current_object, current_object_attributes, attribute, current_attribute_from_user):

        if current_attribute_from_user.lower() in self.get_attribute_of_current_object(attribute, current_object, current_object_attributes):
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
            print "None of the objects found match with the provided attribute"
            #self.tiago.talk("None of the objects found match with the provided attribute")
        # Index of each object remaining in the bounding box
        index = 0
        for match in self.total_matches:
            if match == 0:
                # Updating eliminated objects
                eliminated_objects_indices.append(index)
                
            index +=1

        eliminated_objects_indices.reverse()
        for elemination_index in eliminated_objects_indices:
            eliminated_object = self.objects_within_pointing_bounding_box.pop(elemination_index)
            self.eliminated_objects.append(eliminated_object.get('name'))

        
    def gather_user_response(self, attribute):
        # Stores a list of valid responses
        valid_responses = self.list_of_attributes.get(attribute)

        if not attribute == "position":
            self.tiago.talk("Could you please tell me the " + attribute + " of the object you are pointing at?" )
        else:
            self.reference_object = self.select_reference_object()
            if self.reference_object.get('unique_feature') is not "distance":
                self.tiago.talk("Could you please tell me the direction of the object you are pointing at, in relation to the " + str(self.reference_object.get('unique_feature')) + " object?")
            else:
                self.tiago.talk("Could you please tell me the direction of the object you are pointing at, in relation to the object closest to you?, Which I understand, is a " + str(self.reference_object.get('name')))
                self.tiago.talk("Please use your palm to answer this question, if the object you are pointing at, is on the right side, show your right palm, else show your left palm!")
                # user_response = self.util.classify_each_hand()
                # return user_response
        

        user_response = self.tiago.get_data_from_user("speech", valid_responses, attribute) # request_type, valid_responses, type_of_data
        return user_response


    def compare_all_objects_with_chosen_attribute(self, attribute):
        ## LOOP FOR EACH OBJECT FIRST AND THEN FOR EACH FEATURE!

        # Will be used to store all the objects compared for the current feature
        compared_objects = []
        indices_for_attribute_match = []
        self.total_matches = np.array([])

        current_attribute_from_user = self.gather_user_response(attribute)

        index = 0
        while index < len(self.objects_within_pointing_bounding_box):
            # IF - ELSE block to ensure only objects capable of disambiguation are used for this state
            if self.objects_within_pointing_bounding_box[index].get('name') in self.util.list_of_objects_capable_of_disambiguation:
                current_object = self.objects_within_pointing_bounding_box[index]
                compared_objects = self.compare_current_object_using_attributes_from_database(attribute, current_object, compared_objects, current_attribute_from_user)
                index +=1
            else:
                self.objects_inside_bounding_box_not_compared.append(self.objects_within_pointing_bounding_box.pop(index))
            

        # Updating eliminated objects if attributes do not match:
        self.update_eliminated_objects()
        

        ## Updating the indices of objects in bounding box where a match is found:
        indices_for_attribute_match = np.argwhere(self.total_matches == np.amax(self.total_matches))
        # This is done to remove extra brackets around each element of the list
        indices_for_attribute_match = [val for sublist in indices_for_attribute_match for val in sublist]
        # print indices_for_attribute_match
        
        return indices_for_attribute_match, compared_objects
    
    def notify_status_of_other_objects(self):

        # Called if there were any objects eliminated during disambiguation
        if len(self.eliminated_objects) is not 0:
            print 'The eliminated objects are: '
            print self.eliminated_objects
            self.tiago.talk("The eliminated objects, in order of elimination, are: ")
            for objects in self.eliminated_objects:
                self.tiago.talk(objects)

        # Called if some objects that are not programmed to be disambiguated are found within the bounding box.
        if len(self.objects_inside_bounding_box_not_compared) is not 0:
            self.tiago.talk("The objects that were detected close to the location of pointing, but the ones I am not yet programmed to disambiguate for you are: ")
            for objects in self.objects_inside_bounding_box_not_compared:
                self.tiago.talk(objects.get('name'))


    def find_unique_feature_of_identified_object(self, identified_object):
        unique_feature = ""
        # Loop to identify unique feature of identified object
        ## Note: THIS METHOD HAS TWO LIMITATIONS:
        ## IT ONLY FINDS UNIQUE FEATURES FROM THOSE THAT HAVE BEEN STATED IN THE DATABASE FOR THE OBJECT
        ## IT ALSO DOES ONLY FINDS ITS UNIQUE FEATURE IN RELATION TO OTHER OBJECTS THAT ARE CAPABLE OF DISAMBIGUATION AND DOES NOT FIND COMPARISONS TO OTHER OBJECTS
        ## TO DO SO COMPUTER VISION METHODS MUST BE USED TO FIND ATTRIBUTES SUCH AS SHAPE AND COLOUR!!!
        for current_object in self.objects_with_attributes:
            if current_object.get('name') == identified_object:
                for key in current_object:
                    if not key is 'name': # TO AVOID MATCHES FOR ATTRIBUTE NAMES
                        if current_object[key] in self.unique_features:
                            unique_feature = current_object[key]


        if not len(unique_feature) == 0:
            print("Unique feature of this object is: " + str(unique_feature))
            self.tiago.talk("The unique feature of this object is that it is " + str(unique_feature))
        else:
            print("The object found has no unique attributes!")
            self.tiago.talk("The object found has no unique attributes!")



    def disambiguate_until_object_identified(self):

        self.tiago.talk("Please only refer to the objects that were notified to you earlier, as objects found close to the location of pointing, while answering these questions")

        for attribute in self.attributes:

            indices_for_attribute_match, compared_objects = self.compare_all_objects_with_chosen_attribute(attribute)

            # The == condition ensures that even if it doesnt match for all objects it still goes to the next questions
            if len(indices_for_attribute_match) == 1:

                ## Responding with disambiguation results

                identified_object = compared_objects[indices_for_attribute_match[0]].get('name')
                print identified_object
                self.tiago.talk("The identified object is a " + str(identified_object))
                self.find_unique_feature_of_identified_object(identified_object)
                world_coordinate_of_identified_object = identified_object.get("world_coordinate")
                print("The identified object can be found, relative to the world map, at " + str(world_coordinate_of_identified_object))
                self.tiago.talk("The identified object can be found, relative to the world map, at " + str(world_coordinate_of_identified_object))
                self.notify_status_of_other_objects()

                return

        
        # Code run if diambiguation couldn't find a unique object to suit the descriptions
        self.tiago.talk("Sorry but I couldn't disambiguate the object for you, given the provided descriptions")
        self.notify_status_of_other_objects()


    # PUT TEXT AND SPEECH BOTH IN TIAGO.PY

    def find_closest_object_in_bounding_box_to_user(self):
        ## TO DO
        self.person_head_world_coordinate
        array_of_world_coordinates = []
        for current_object in self.objects_within_pointing_bounding_box:
            current_object_world_coordinate = current_object.get('world_coordinate')
            # print current_object_world_coordinate
            # # Storing only the x and y dimentions as the height does not matter here for distance
            # current_object_world_coordinate_2d = [current_object_world_coordinate[0],current_object_world_coordinate[1]]
            array_of_world_coordinates.append(current_object_world_coordinate)
        
        # # Finds the arg of the point closest to the person. Storing only the x and y dimentions as the height does not matter here for distance
        # person_head_world_coordinate_2d = [self.person_head_world_coordinate[0],self.person_head_world_coordinate[1]]

        # Making sure both are arrays with equal number of columns:
        person_head_world_coordinate = np.array(self.person_head_world_coordinate).reshape(1, -1)
        array_of_world_coordinates = np.array(array_of_world_coordinates)

        # print self.person_head_world_coordinate
        # print array_of_world_coordinates

        # Finds the closest object to the person
        closest_index = distance.cdist(array_of_world_coordinates, person_head_world_coordinate, 'euclidean').argmin()

        # Saving the details in reference_object
        closest_object = {
            "name": self.objects_within_pointing_bounding_box[closest_index].get('name'),
            "unique_feature": "distance",
            "world_coordinate": self.objects_within_pointing_bounding_box[closest_index].get('world_coordinate')
        }
        print ("The closest object to the person is: ")
        print closest_object.get('name')
        return closest_object


    def get_unique_features(self):

        self.objects_within_pointing_bounding_box_with_attributes = []
        
        for current_object in self.objects_within_pointing_bounding_box:
            for object_id in range(len(self.objects_with_attributes)):
                if self.objects_with_attributes[object_id].get('name') == current_object.get('name'):
                    # This buffer list created so that the objects_within_pointing_bounding_box_with_attributes is not a dict and is just a list with all the attributes for the object
                    current_object_with_attributes = [self.objects_with_attributes[object_id].get('name')]
                    for attribute in self.attributes:
                        current_object_with_attributes.append(self.objects_with_attributes[object_id].get(attribute))

                    self.objects_within_pointing_bounding_box_with_attributes.append(current_object_with_attributes)
                #else:
                    ## TO DO IF OBJECT ATTRIBUTES ARE NOT AVAILABLE

        #unique_features = list(set().union(self.objects_within_pointing_bounding_box_with_attributes))


        # # Combining all elements from the list of lists into one list:
        # merged_objects_within_pointing_bounding_box_with_attributes = []

        # for item in self.objects_within_pointing_bounding_box_with_attributes:
        #     merged_objects_within_pointing_bounding_box_with_attributes += item
        
        # # Finding only unique values in these lists:

        # # insert the list to the set
        # list_set = set(merged_objects_within_pointing_bounding_box_with_attributes)
        # # convert the set to the list
        # unique_features = (list(list_set))

        # These functions count the number of times an attribute is repeated in objects found within pointing bounding box and if it is not repeated it is a unique feature
        counts = Counter(chain.from_iterable(self.objects_within_pointing_bounding_box_with_attributes))
        unique_features = [k for k, c in counts.items() if c == 1]

        # Removing all elements that are object names as these are unique objects found, but not features:
        for element in self.util.list_of_objects_capable_of_disambiguation:
            if element in unique_features:
                unique_features.remove(element)
        
        print ("The unique features are: ")
        print unique_features


        return unique_features



    def execute(self, userdata):
        rospy.loginfo('ObjectDisambiguation state executing')

        #self.tiago.talk("My name is Ahmed and I am the robo maker")
        self.objects_within_pointing_bounding_box = rospy.get_param('/objects_within_pointing_bounding_box')
        self.person_head_world_coordinate = rospy.get_param('/person_head_world_coordinate')
        self.current_table = rospy.get_param("/current_table")
        self.unique_features = self.get_unique_features()
        self.closest_object = self.find_closest_object_in_bounding_box_to_user()

        # print objects_within_pointing_bounding_box
        # print objects_within_pointing_bounding_box[0].get('name')

        self.attributes_from_user = []

        # self.dummy_attributes_from_user = {
        #         'colour':  'yellow',
        #         'type':    'fresh',
        #         'texture': 'smooth',
        #         'size':    'long',
        #         'shape':   'curved'
        #     }
        

        self.disambiguate_until_object_identified()
                



    # ## Return Object With Highest Confidence
    # for object in len(indices_for_attribute_match):
    #     detection_confidence.append(list_of_objects_within_bounding_box[indices_for_attribute_match[object]].confidence)
    # highest_confidence_index = indices_for_attribute_match[np.argmax(detection_confidence)]
    # identified_object = list_of_objects_within_bounding_box[highest_confidence_index]
    # return identified_object



        # To destroy cv2 window at the end of state
        cv2.destroyAllWindows()

        return 'outcome1'

