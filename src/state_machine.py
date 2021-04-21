#!/usr/bin/env python
import rospy
import cv2
from smach import State, StateMachine

from utilities import ClassifyObjects, Interaction, Util, Move
# Tried doing this so that gazebo dooesnt eat up all the ram needed for openpose
#from utilities import GetPoseBeforeGazebo
from states import ApproachPerson, PointingLocationDetection, ApproachPointedObject, PointedObjectDetection, ObjectDisambiguation, LookAtPersonForInteraction



def wait_for_command(interaction):
    interaction.talk("I am at your service, please give me a command." )
    success = False
    table_types = ["dining","lounge","kitchen","study"]
    while success == False:
        user_response = interaction.get_start_command("speech") # request_type, valid_responses, type_of_data
        words = user_response.lower().split()
        if ("table" in words[0:]):
            if (("what" in words[0:])or("name" in words[0:]))and(("this" in words[0:])or("the" in words[0:])or("that" in words[0:])):
                table_type = words[words.index("table")-1]
                if table_type in table_types:
                    table  = table_type + " table"
                    print (table_type + " table")
                    success = True
                else:
                    interaction.talk("Sorry, I didn't quite get that, could you please repeat that." )
                    success = False
            else:
                interaction.talk("Sorry, I am not able to perform that request. Please let me know if I can help you with anything else." )
                success = False
        else:
            interaction.talk("Sorry, I didn't quite get that, could you please repeat that." )
            success = False

    return table


# main
def main():
    rospy.init_node("state_machine")

    # default dataset for yolo3 is coco unless change needed for more accurate detection from a particular dataset, which is passed here.
    # openimages can be used which offers almost all common objects for detection
    classify_objects = ClassifyObjects(dataset="coco")
    #creates an instance of move class to move robot across the map and perform physical actions
    move = Move()
    #creates an instance of interaction class to interact with the user and perform physical actions
    interaction = interaction()
    # Lift tiago's torso and set head to default
    move.lift_torso_head_default(True)
    #creates an instance of util class to use featues such as extract attributes of objects from yaml file and transform point frames
    util = Util()

    table = wait_for_command(interaction)


    ## REMOVE FOLLOWING CODE WHEN RUNNING POSE DETECTION AS THESE ARE DEFAULT VALUES TO USE FOR TESTING
    intersection_point_2d = [388, 239]
    rospy.set_param("/intersection_point_2d", intersection_point_2d)
    intersection_point_3d = [0.2793646814287425, -0.004621289580974977, 2.1561762792235117]
    rospy.set_param("/intersection_point_3d", intersection_point_3d)
    intersection_point_world = [-2.195603797301315, -9.08019820397351, 1.0689875402030786]
    rospy.set_param("/intersection_point_world", intersection_point_world)
    person_head_world_coordinate = [-1.402642251022807, -8.916499175910454, 1.792557797900345]
    rospy.set_param("/person_head_world_coordinate", person_head_world_coordinate)
    radius_of_pointing = 0.18
    rospy.set_param("/radius_of_pointing", radius_of_pointing)
    

    # Create a SMACH state machine
    sm = StateMachine(outcomes=["outcome1", "end"])
    # Open the container

    with sm:
        # Add states to the container
        StateMachine.add("approach_person", ApproachPerson(classify_objects, interaction, util, move, table), transitions={"outcome1":"detect_pointing_location", "outcome2": "detect_pointing_location"})
        StateMachine.add("detect_pointing_location", PointingLocationDetection(interaction, util), transitions={"outcome1":"approach_object", "outcome2": "end"})
        StateMachine.add("approach_object", ApproachPointedObject(interaction, util, move), transitions={"outcome1":"detect_pointed_object", "outcome2": "end"})
        StateMachine.add("detect_pointed_object", PointedObjectDetection(classify_objects, interaction, util), transitions={"outcome1":"look_at_person_for_interaction", "outcome2": "end"})
        StateMachine.add("look_at_person_for_interaction", LookAtPersonForInteraction(interaction, move), transitions={"outcome1":"disambiguate_objects", "outcome2": "disambiguate_objects"})
        StateMachine.add("disambiguate_objects", ObjectDisambiguation(interaction, util), transitions={"outcome1":"end", "outcome2": "end"})
        sm.execute()
    
    #cv2.waitKey(0)

    #rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("State Machine terminated...")