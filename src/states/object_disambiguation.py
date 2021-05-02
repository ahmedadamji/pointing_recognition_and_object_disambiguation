#!/usr/bin/env python

import rospy
import actionlib
from smach import State


from pointing_recognition.srv import HandClassification

import cv2
import numpy as np
import math
from sympy import Point3D

from scipy.spatial import distance

from collections import Counter
from itertools import chain




class ObjectDisambiguation(State):
    def __init__(self, interaction, util):
        #rospy.loginfo("ObjectDisambiguation state initialized")
        State.__init__(self, outcomes=["outcome1","outcome2"])

        #creates an instance of interaction class to interact with the user
        self.interaction = interaction
        #creates an instance of util class to use features such as extract attributes of objects from yaml file and transform point frames
        self.util = util

        # Stores details of objects that have been eliminated during disambiguation
        self.eliminated_objects = []
        # Stores details of objects that are not part of disambiguation but were found inside the bounding box
        self.objects_inside_bounding_region_not_compared = []
        # Stores all matches from current attribute check, used to decide eliminated objects as well as which objects go to the next stage
        self.total_matches = np.array([])

        # Stores the types of attributes in order of use for disambiguation
        self.attributes = "position", "colour", "smooth", "shape"
        # Stores the possible directions in terms of compass coordinates to use as part of disambiguating objects
        self.compass_directions = ["north", "east", "south", "west", "north", "centre"]
        # Stores the possible directions to use as part of disambiguating objects
        self.standard_directions = ["behind", "right", "front", "left", "behind", "that"]
        # Number of directions that can be referenced will be stored here, if the reference object is closest to the person, then only 2 (right or left) 
        # directions make sense, the default value is 4
        self.directions = 4

        self.objects_with_attributes = self.util.object_attributes
        self.list_of_attributes = self.util.list_of_attributes

    def convert_standard_directions_to_compass_directions(self, direction_of_current_object):
        #Converting north, south, east, west TO behind, front , right , left
        if direction_of_current_object.lower() in self.standard_directions:
            index = self.standard_directions.index(direction_of_current_object.lower())
            return self.compass_directions[index]
        else:
            return direction_of_current_object


    def calculate_compass_direction_between_two_points(self, point_of_interest, view_point_transformed, reference_point = [0,0]):
        ## REFERENCE: https://www.analytics-link.com/post/2018/08/21/calculating-the-compass-direction-between-two-points-in-python

        deltaX = point_of_interest[0] - reference_point[0]
        deltaY = point_of_interest[1] - reference_point[1]
        angle_between_points = math.atan2(deltaX, deltaY)/math.pi*180
        distance_between_points = math.hypot(deltaX, deltaY)

        # Test it out if this distance of 0.02 is useful or if i should change it back to 0
        if distance_between_points < 0.001:
            compass_direction = self.compass_directions[5]
            return compass_direction

        
        elif angle_between_points < 0:
            angle_between_points = 360 + angle_between_points

        else:
            angle_between_points = angle_between_points

        direction_index = 0
        if self.directions == 4:
            max_angle = 90
            direction_index = (int(round(angle_between_points / max_angle)))
            #print direction_index
        elif self.directions == 2:
            # https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located
            if (((float(point_of_interest[0]) - float(0))*(float(view_point_transformed[1]) - float(0))) - ((float(point_of_interest[1]) - float(0))*(float(view_point_transformed[0]) - float(0)))) > 0.0:
                direction_index = 3
            elif (((float(point_of_interest[0]) - float(0))*(float(view_point_transformed[1]) - float(0))) - ((float(point_of_interest[1]) - float(0))*(float(view_point_transformed[0]) - float(0)))) < 0.0:
                direction_index = 1
            elif (((float(point_of_interest[0]) - float(0))*(float(view_point_transformed[1]) - float(0))) - ((float(point_of_interest[1]) - float(0))*(float(view_point_transformed[0]) - float(0)))) == 0.0:
                direction_index = 5
            #print direction_index

        
        compass_direction = self.compass_directions[direction_index]

        return compass_direction
        


    def transfer_coordinate_wrt_person_and_reference_object(self, point_of_interest, reference_point, view_point):
        #http://motion.cs.illinois.edu/RoboticSystems/CoordinateTransformations.html


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

        # Transfering frame for the view point as well.
        wv = [view_point[0],view_point[1],1]
        wv = np.array(wv).reshape(3,1)

        rp = np.matmul(rtw, wp)
        point_of_interest_transformed = [rp[0],rp[1]]

        rv = np.matmul(rtw, wv)
        view_point_transformed = [rv[0],rv[1]]
        
        #print point_of_interest_transformed


        return point_of_interest_transformed, view_point_transformed

    def select_reference_object(self):

        object_with_unique_feature = None
        world_coordinate_of_object_with_unique_feature = None
        
        if len(self.unique_features) is not 0:
            # HERE I CAN USE ANY OF THE UNIQUE FEATURES TO REFERNCE THE OBJECTS
            # using the first element only as there can be more than one unique feature found
            for feature in self.unique_features:
                if not ((feature == "yes") or (feature == "no")):
                    unique_feature = feature
                else:
                    reference_object = self.closest_object
                    # only right or left directions as all of them will essentially be front the nearest object
                    self.directions = 2
            
            unique_feature = self.unique_features[0]
            # print unique_feature
            for current_object in self.objects_within_pointing_bounding_region_with_attributes_for_unique_features:
                #print current_object
                if unique_feature in current_object:
                    object_with_unique_feature = current_object[0]
            for current_object in self.objects_within_pointing_bounding_region:
                if current_object.get("name") == object_with_unique_feature:
                    world_coordinate_of_object_with_unique_feature = current_object.get("world_coordinate")
            reference_object = {
                "name": object_with_unique_feature,
                "unique_feature": unique_feature,
                "world_coordinate":world_coordinate_of_object_with_unique_feature
            }
            self.directions = 4

        else:
            reference_object = self.closest_object
            # only right or left directions as all of them will essentially be front the nearest object
            self.directions = 2
        
        return reference_object
    
    def get_compass_direction(self, current_object):

        
        # Gets the World coordinate of the object:
        current_object_world_coordinate = current_object.get("world_coordinate")

        # Finds the reference object that has a unique attribute compared to the rest and transfers the coordinate frame from the world frame
        # to a reference frame centred around this reference object and aligned with the line of sight of the person answering questions
        reference_object_world_coordinate = self.reference_object.get("world_coordinate")
        # print current_object_world_coordinate
        # print reference_object_world_coordinate
        # print self.person_head_world_coordinate
        current_object_world_coordinate_transformed, view_point_transformed = self.transfer_coordinate_wrt_person_and_reference_object(current_object_world_coordinate, reference_object_world_coordinate, self.person_head_world_coordinate)        

        # Gets the pointing centre point, also the centre point in bounding box, to use as reference for directions
        # centre_point_of_bounding_region = rospy.get_param("/camera_point_after_object_detection_2d")
        compass_direction = self.calculate_compass_direction_between_two_points(current_object_world_coordinate_transformed, view_point_transformed)
        return compass_direction

    def get_attribute_of_current_object(self, attribute, current_object, current_object_attributes):
        #user_response = self.dummy_attributes_from_user.get(attribute)
        # Can add here more conditions for example if attribute needs to be extracted by means of opencv / other means in the future.
        if not(attribute == "position"):
            attribute_of_current_object = current_object_attributes.get(attribute)
        else:
            compass_direction = self.get_compass_direction(current_object)

            # This block of code if just for printing the direction found of the objects in the bounding box and are not essential to the functionality
            index = self.compass_directions.index(compass_direction)
            standard_direction = self.standard_directions[index]
            if compass_direction is not "centre":
                print (current_object.get("name") + " was found in the: " + compass_direction + " / " + standard_direction)
            else:
                print (current_object.get("name") + " was found in the: " + compass_direction)
            
            attribute_of_current_object = [compass_direction, standard_direction]
            
        
        return attribute_of_current_object
        


    def compare_current_object_with_chosen_attribute(self, current_object, current_object_attributes, attribute, user_response):
        # print self.get_attribute_of_current_object(attribute, current_object, current_object_attributes)

        if user_response.lower() in self.get_attribute_of_current_object(attribute, current_object, current_object_attributes):
            match = 1
            print attribute + " attribute matches"
        else:
            match = 0
            # self.eliminated_objects.append(current_object_attributes.get("name"))
            print attribute + " attribute does not match"
        
        return match
    
    
    def compare_current_object_using_attributes_from_database(self, attribute, current_object, compared_objects, user_response):

        for object_id in range(len(self.objects_with_attributes)):
            if self.objects_with_attributes[object_id].get("name") == current_object.get("name"):
                print "Current object being compared: " + self.objects_with_attributes[object_id].get("name")
                compared_objects.append(current_object)

                match = self.compare_current_object_with_chosen_attribute(current_object, self.objects_with_attributes[object_id], attribute, user_response)

                self.total_matches = np.array(np.append(self.total_matches, [match], axis = 0))
                    
            else:
                continue

        return compared_objects
        

    def update_eliminated_objects(self):
        # This is done so that eliminated objects are only updated if they there is at least one match for the current atrribute
        
        # Stores indices of objects from within the bounding box to be eliminated
        eliminated_objects_indices = []
        if all([ v == 0 for v in self.total_matches]):
            print "None of the objects found match with the provided attribute"
            #self.interaction.talk("None of the objects found match with the provided attribute")
        # Index of each object remaining in the bounding box
        index = 0
        for match in self.total_matches:
            if match == 0:
                # Updating eliminated objects
                eliminated_objects_indices.append(index)
                
            index +=1

        eliminated_objects_indices.reverse()
        for elemination_index in eliminated_objects_indices:
            eliminated_object = self.objects_within_pointing_bounding_region.pop(elemination_index)
            self.eliminated_objects.append(eliminated_object.get("name"))

        
    def gather_user_response(self, attribute):
        # Stores a list of valid responses
        valid_responses = self.list_of_attributes.get(attribute)

        if not attribute == "position":
            if attribute == "smooth":
                self.interaction.talk("Would you say that the texture of the object is silky smooth?")
            else:
                self.interaction.talk("Could you please tell me the " + attribute + " of the object you are pointing at?" )
        else:
            self.reference_object = self.select_reference_object()
            if self.reference_object.get("unique_feature") is not "distance":
                if self.reference_object.get("unique_feature") is not "smooth":
                    self.interaction.talk("Could you please tell me the direction of the object you are pointing at, in relation to the. " + str(self.reference_object.get("unique_feature")) + " object?")
                else:
                    self.interaction.talk("Could you please tell me the direction of the object you are pointing at, in relation to the object with a smooth texture?")

            else:
                self.interaction.talk("In relation to the object that is the closest to you, from those displayed within the circle earlier, is the object you were referring to on the right or left")
                self.interaction.talk("Please show your hand to answer this question. If your answer was right, show your right hand, else show your left hand!")
                # user_response = self.util.classify_each_hand()
                # return user_response

                try:
                    rospy.loginfo("waiting for classify_hands service")
                    rospy.wait_for_service("/classify_hands")
                    rospy.loginfo("connected to classify_hands service")

                    # running openpose detection
                    try:
                        hand_classification = rospy.ServiceProxy("/classify_hands", HandClassification)
                        self.response = hand_classification()

                    except rospy.ServiceException as e:
                        print("Hand detection failed")
                        rospy.logwarn(e)

                    user_response = self.response.hand


                    #print user_response
                    self.interaction.talk("I see that you have responded with your " + user_response + " hand")
                    return user_response


                except rospy.ROSInterruptException:
                    pass
                except rospy.ServiceException as ex:
                    rospy.logwarn("service call classify_hands failed")
                    rospy.logwarn(ex)
                except rospy.ROSException as ex:
                    rospy.logwarn("timed out waiting for classify_hands service")
                    rospy.logwarn(ex)


        user_response = self.interaction.get_data_from_user("speech", valid_responses, attribute) # request_type, valid_responses, type_of_data
        return user_response

    
    def is_not_duplicate(self, features):
        result = False
        if len(features) > 0 :
            result = all(elem == features[0] for elem in features)
        #print result
        if (result):
            return False
        else:
            return True


    def compare_all_objects_with_chosen_attribute(self, attribute):
        ## LOOP FOR EACH OBJECT FIRST AND THEN FOR EACH FEATURE!

        # Will be used to store all the objects compared for the current feature
        compared_objects = []
        indices_for_attribute_match = []
        self.total_matches = np.array([])


        features = []
        #print self.objects_within_pointing_bounding_region_with_attributes
        #print self.objects_within_pointing_bounding_region_with_attributes
        for current_object in self.objects_within_pointing_bounding_region_with_attributes:
            attribute_index = self.attributes.index(attribute)
            #indices = [i for i, x in enumerate(self.attributes) if x == attribute]
            #print "indices: " + str(indices)
            features.append(current_object[0][attribute_index])
            #print features
            #print current_object

        #print attribute
        #print(features)
        # print self.is_not_duplicate(features)
        #print (attribute == "position")

        if self.is_not_duplicate(features) or (attribute == "position"):
            user_response = self.gather_user_response(attribute)

            index = 0
            while index < len(self.objects_within_pointing_bounding_region):
                # IF - ELSE block to ensure only objects capable of disambiguation are used for this state
                if self.objects_within_pointing_bounding_region[index].get("name") in self.util.list_of_objects_capable_of_disambiguation:
                    current_object = self.objects_within_pointing_bounding_region[index]
                    #print current_object
                    compared_objects = self.compare_current_object_using_attributes_from_database(attribute, current_object, compared_objects, user_response)
                    index +=1
                    #print self.objects_within_pointing_bounding_region
                else:
                    self.objects_inside_bounding_region_not_compared.append(self.objects_within_pointing_bounding_region.pop(index))
                    #self.objects_within_pointing_bounding_region_with_attributes.pop(index)
                

            # Updating eliminated objects if attributes do not match:
            self.update_eliminated_objects()
            

            ## Updating the indices of objects in bounding box where a match is found:
            if np.amax(self.total_matches) >= 1:
                indices_for_attribute_match = np.argwhere(self.total_matches == np.amax(self.total_matches))
                # This is done to remove extra brackets around each element of the list
                indices_for_attribute_match = [val for sublist in indices_for_attribute_match for val in sublist]
            
            #print indices_for_attribute_match
            
            return indices_for_attribute_match, compared_objects
        
        else:
            return indices_for_attribute_match, compared_objects


    def find_unique_feature_of_identified_object(self, identified_object):
        unique_feature = ""
        # Loop to identify unique feature of identified object
        ## Note: THIS METHOD HAS TWO LIMITATIONS:
        ## IT ONLY FINDS UNIQUE FEATURES FROM THOSE THAT HAVE BEEN STATED IN THE DATABASE FOR THE OBJECT
        ## IT ALSO DOES ONLY FINDS ITS UNIQUE FEATURE IN RELATION TO OTHER OBJECTS THAT ARE CAPABLE OF DISAMBIGUATION AND DOES NOT FIND COMPARISONS TO OTHER OBJECTS
        ## TO DO SO COMPUTER VISION METHODS MUST BE USED TO FIND ATTRIBUTES SUCH AS SHAPE AND COLOUR!!!
        for current_object in self.objects_with_attributes:
            if current_object.get("name") == identified_object:
                for key in current_object:
                    if (not key is "name") and (not key is "shape") and (not key is "shape"): # TO AVOID MATCHES FOR ATTRIBUTE NAMES AND SHAPES
                        if current_object[key] in self.unique_features:
                            unique_feature = current_object[key]
                            attribute = key
                            return unique_feature, attribute
                for colour in current_object['colour']:
                    if colour in self.unique_features:
                        unique_feature = colour
                        attribute = 'colour'
                        return unique_feature, attribute
                for shape in current_object['shape']:
                    if shape in self.unique_features:
                        unique_feature = shape
                        attribute = 'shape'
                        return unique_feature, attribute
        return [],""

    
    def notify_status_of_other_objects(self):

        # Called if there were any objects eliminated during disambiguation
        if len(self.eliminated_objects) is not 0:
            print "The eliminated objects are: "
            #print self.eliminated_objects
            eliminated_objects = ""
            #self.interaction.talk("The eliminated objects, in order of elimination, are: ")
            for i in range(len(self.eliminated_objects)):
                if (i == (len(self.eliminated_objects)-1)and(len(self.eliminated_objects)>2)):
                    eliminated_objects += " and "
                object_name = self.eliminated_objects[i]
                eliminated_objects += object_name
                if not ((i == (len(self.eliminated_objects)-1)) or (i == (len(self.eliminated_objects)-2))):
                    eliminated_objects += ", "
            print eliminated_objects





    def disambiguate_until_object_identified(self):
        unique_feature = []

        self.interaction.talk("While answering these questions, please only refer to the objects displayed to you previously within a circle.")

        for attribute in self.attributes:
            print attribute
            try:
                self.indices_for_attribute_match, compared_objects = self.compare_all_objects_with_chosen_attribute(attribute)
            except Exception as e:
                # This condition removes the need to ask the user for an input when the answer does not have any value for a particular attribute (i.e. all objects have the same value)
                continue

            # The == condition ensures that if there is only one match, it doesnt got o the next question

            pos_index = self.attributes.index("position")
            att_index = self.attributes.index(attribute)

            if len(self.indices_for_attribute_match) == 1:

                ## Responding with disambiguation results

                identified_object = compared_objects[self.indices_for_attribute_match[0]]
                #print identified_object.get("name")
                self.interaction.talk("The identified object is called " + str(identified_object.get("name")))
                unique_feature, attribute = self.find_unique_feature_of_identified_object(identified_object.get("name"))


                if not len(unique_feature) == 0:
                    if (not attribute is "smooth"):
                        print("The unique feature of this object is that its " + attribute + " is " + str(unique_feature))
                        self.interaction.talk("The unique feature of this object is that its " + attribute + " is " + str(unique_feature))
                    else:
                        if unique_feature == "no":
                            print("The unique efature of this object is that it's texture is not smooth")
                            self.interaction.talk("The unique feature of this object is that it's texture is not smooth")
                        else:
                            print("The unique feature of this object is that it's texture is smooth")
                            self.interaction.talk("The unique feature of this object is that it's texture is smooth")


                else:
                    print("The object found has no unique attributes!")
                    self.interaction.talk("The object found has no unique attributes! I see why you were confused!")


                world_coordinate_of_identified_object =  np.around(np.array(identified_object.get("world_coordinate")),2) # World coordinate in 2dp
                print("The identified object can be found, relative to the world map, at " + str(world_coordinate_of_identified_object))
                #self.interaction.talk("The identified object can be found, relative to the world map, at " + str(world_coordinate_of_identified_object))
                self.notify_status_of_other_objects()

                return
            # This condition ensures that as soon as none of the objects match the provided attributes, it stops disambiguation.
            elif len(self.indices_for_attribute_match) == 0:
                # Code run if diambiguation couldn't find a unique object to suit the descriptions
                self.interaction.talk("Sorry but none of the objects I could identify match this attribute")
                self.notify_status_of_other_objects()
                return
            
                    
            elif ((len(unique_feature) == 0) and (att_index>pos_index)):

                previous_object_name = compared_objects[self.indices_for_attribute_match[0]].get("name")
                count = 1
                for i in range(len(self.indices_for_attribute_match)-1):
                    object_name = compared_objects[self.indices_for_attribute_match[i+1]].get("name")
                    if previous_object_name == object_name:
                        count += 1
                    previous_object_name = object_name
                if count == len(self.indices_for_attribute_match):
                    identified_object = compared_objects[self.indices_for_attribute_match[0]]
                    self.interaction.talk("The object you were pointing at was a " + identified_object.get("name"))
                    self.interaction.talk("But I am not able to determine which one you were pointing at, as there were multiple of these close to each other")
                    self.notify_status_of_other_objects()
                    return


        # Code run if diambiguation couldn't find a unique object to suit the descriptions
        self.interaction.talk("Sorry but I couldn't disambiguate the object for you, given the provided descriptions")
        # Notify of other objects that are not yet programmed to disambiguated
        if len(self.objects_inside_bounding_region_not_compared) is not 0:
            self.interaction.talk("The object you were pointing at might be a")
            for objects in self.objects_inside_bounding_region_not_compared:
                self.interaction.talk(objects.get("name"))
            self.interaction.talk("But I am not programmed to help you with this yet.")


        self.notify_status_of_other_objects()

    def find_closest_object_in_bounding_region_to_user(self):
        ## TO DO
        self.person_head_world_coordinate
        array_of_world_coordinates = []
        for current_object in self.objects_within_pointing_bounding_region:
            current_object_world_coordinate = current_object.get("world_coordinate")
            # print current_object_world_coordinate
            array_of_world_coordinates.append(current_object_world_coordinate)
        
        # Making sure both are arrays with equal number of columns:
        person_head_world_coordinate = np.array(self.person_head_world_coordinate).reshape(1, -1)
        array_of_world_coordinates = np.array(array_of_world_coordinates)

        # print self.person_head_world_coordinate
        # print array_of_world_coordinates

        # Finds the closest object to the person
        closest_index = distance.cdist(array_of_world_coordinates, person_head_world_coordinate, "euclidean").argmin()

        # Saving the details in reference_object
        closest_object = {
            "name": self.objects_within_pointing_bounding_region[closest_index].get("name"),
            "unique_feature": "distance",
            "world_coordinate": self.objects_within_pointing_bounding_region[closest_index].get("world_coordinate")
        }
        print ("The closest object to the person is: ")
        print closest_object.get("name")
        return closest_object

    def get_key(self, val):
        for key, list_of_val in self.list_of_attributes.items():
            #print list_of_val
            if val in list_of_val:
                return key
    

    def get_objects_within_pointing_bounding_region_with_attributes(self):

        objects_within_pointing_bounding_region_with_attributes = []
        objects_within_pointing_bounding_region_with_attributes_for_unique_features = []
        
        for current_object in self.objects_within_pointing_bounding_region:
            for object_id in range(len(self.objects_with_attributes)):
                if self.objects_with_attributes[object_id].get("name") == current_object.get("name"):

                    # This buffer list created so that the objects_within_pointing_bounding_region_with_attributes is not a dict and is just a list with all the attributes for the object
                    current_object_with_attributes = []
                    current_object_with_attributes_for_unique_features = []
                    current_object_with_attributes_for_unique_features = [self.objects_with_attributes[object_id].get("name")]
                    
                    for attribute in self.attributes:

                        current_object_with_attributes.append(self.objects_with_attributes[object_id].get(attribute))

                        ## NEED TO DO THIS BECAUSE THERE ARE MULTIPLE TYPES OF SHAPES AND COLOURS
                        if (not attribute == "shape") and (not attribute == "colour"):
                            current_object_with_attributes_for_unique_features.append(self.objects_with_attributes[object_id].get(attribute))
                        elif (attribute == "shape"):
                            for shape in self.objects_with_attributes[object_id].get(attribute):
                                current_object_with_attributes_for_unique_features.append(shape)
                        elif (attribute == "colour"):
                            for colour in self.objects_with_attributes[object_id].get(attribute):
                                current_object_with_attributes_for_unique_features.append(colour)

                    objects_within_pointing_bounding_region_with_attributes.append(current_object_with_attributes)
                    objects_within_pointing_bounding_region_with_attributes_for_unique_features.append(current_object_with_attributes_for_unique_features)
                    
                    
        return objects_within_pointing_bounding_region_with_attributes, objects_within_pointing_bounding_region_with_attributes_for_unique_features

    def get_unique_features(self):
        self.objects_within_pointing_bounding_region_with_attributes, self.objects_within_pointing_bounding_region_with_attributes_for_unique_features = self.get_objects_within_pointing_bounding_region_with_attributes()

        # These functions count the number of times an attribute is repeated in objects found within pointing bounding box and if it is not repeated it is a unique feature
        counts = Counter(chain.from_iterable(self.objects_within_pointing_bounding_region_with_attributes_for_unique_features))
        unique_features = [k for k, c in counts.items() if c == 1]
        
        
        unique_features_attributes = []
        if len(unique_features) > 0:

            for unique_feature in unique_features:
                if not self.get_key(unique_feature) == "shape":
                    unique_features_attributes.append(self.get_key(unique_feature))
                    
        
        print ("The unique features of objects on the table are : ")
        print unique_features
        print ("The attributes for unique features of objects on the table are : ")
        print unique_features_attributes

        counts = Counter(unique_features_attributes)
        # First ordering in terms of max frequency of attribute
        reordered_attributes = sorted(unique_features_attributes, key=counts.get, reverse=True)
        # Removing duplicate values
        reordered_attributes = list(set(reordered_attributes))
        for attribute in self.attributes:
            if (not attribute in reordered_attributes) and (not attribute == "shape"):
                reordered_attributes.append(attribute)
        ## Shape is added last because it has multiple options associated and with it automatically become first in the reordered list
        reordered_attributes.append("shape")
        self.attributes = reordered_attributes
        print self.attributes
        
        # Done again as the order of attributes has now changed due to intent of maximising information gain
        self.objects_within_pointing_bounding_region_with_attributes = self.get_objects_within_pointing_bounding_region_with_attributes()

        return unique_features



    def execute(self, userdata):
        rospy.loginfo("ObjectDisambiguation state executing")

        self.objects_within_pointing_bounding_region = rospy.get_param("/objects_within_pointing_bounding_region")
        self.person_head_world_coordinate = rospy.get_param("/person_head_world_coordinate")
        self.unique_features = self.get_unique_features()
        self.closest_object = self.find_closest_object_in_bounding_region_to_user()

        # print objects_within_pointing_bounding_region
        # print objects_within_pointing_bounding_region[0].get("name")

        self.attributes_from_user = []

        # self.dummy_attributes_from_user = {
        #         "colour":  "yellow",
        #         "type":    "fresh",
        #         "texture": "smooth",
        #         "size":    "long",
        #         "shape":   "curved"
        #     }
        

        self.disambiguate_until_object_identified()


        # To destroy cv2 window at the end of state
        #cv2.destroyAllWindows()

        return "outcome1"

