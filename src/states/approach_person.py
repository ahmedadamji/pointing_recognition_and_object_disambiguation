#!/usr/bin/env python
import rospy
import actionlib

from smach import State
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
import tf
import math


class ApproachPerson(State):
    def __init__(self, classify_objects, interaction, util, move, table):
        #rospy.loginfo("ApproachPerson state initialized")
        
        State.__init__(self, outcomes=["outcome1","outcome2"])
        
        # creates an instance of classify_objects class to classify yolo detections
        self.classify_objects = classify_objects
        #creates an instance of interaction class to interact with the user and perform physical actions
        self.interaction = interaction
        #creates an instance of util class to transform point frames
        self.util = util
        #creates an instance of move class to move robot across the map
        self.move = move
        #Stores the name of the table requested to be approached for poinitng
        self.table = table


    def get_table(self):
        for table_id in range(0, len(self.tables)):
            status = self.tables[table_id].get("status")
            if status == "not checked":
                table_name = self.tables[table_id].get("name")
                if table_name == self.table:
                    print table_name + " is the target location."
                    rospy.set_param("/current_table", self.tables[table_id])
                    self.tables[table_id]["status"] = "checked"
                    return
        print("This table has already been checked")
        return "table_checked"

    def detect_person(self):
        self.classify_objects.subscribe_to_vision_messages()
        yolo_detections = self.classify_objects.yolo_object_detection()
        # Not finding segmentations if no objects detected using yolo
        if not len(yolo_detections):
            return None
        #self.classify_objects.yolo_get_object_coordinates()

        for index in range(len(yolo_detections)):
            if (yolo_detections[index].name.lower() == "person"):
                x,y,w,h = yolo_detections[index].xywh
                if ((w*h) > 65000):
                    print("A person was found in frame")
                    return True
        print("No person was found in frame")
        return False

    def move_to_table(self,current_table):
        #location = rospy.get_param("/pointing_person_approach")
        location = current_table.get("table_default_location")

        # Sending Move class the location to move to, and stores result in movebase
        movebase = self.move.move_base(location)
        #self.move.rotate_around_base(-20)
        if movebase == True:
            print("Reached the "  + current_table.get("name"))
        else:
            # INSERT HERE THE ACTION IF GOAL NOT ACHIEVED
            self.interaction.talk("I have not been able to reach the " + current_table.get("name") )


    def check_person_around_table(self):
        degrees = 0
        while (degrees <= 43):


            # Sending Move class the angle to rotate about, and stores result in rotate, and also returns the final movebase location
            rotate, location = self.move.rotate_around_base(degrees)
            if rotate == True:
                print("The robot has rotated around the base by " + str(degrees) + " degrees anticlockwise")
            else:
                # INSERT HERE THE ACTION IF GOAL NOT ACHIEVED
                print("The robot has not been able to rotate around it's base")

            if self.detect_person():
                print("Person was found at this table")
                # Saves the location where the pwerson was found to rosparm to use this later while looking back at person for gathering responses
                rospy.set_param("/pointing_person_approach_orientation", [location.get("orientation")[0].item(), location.get("orientation")[1].item(), location.get("orientation")[2].item(), location.get("orientation")[3].item()])
                # rospy.set_param("/pointing_person_approach", location)

                return True
            degrees += 43

                
        print("No Person was found at this table")
        return False



    def execute(self, userdata, wait=True):
        rospy.loginfo("ApproachPerson state executing")

        # Collects the details of tables in the environment from the util class and saves in self.tables
        self.tables = self.util.tables

        # CHANGE TABLE NAME HERE:
        self.interaction.talk("I am now going to approach the " +  self.table + ", to help you identify an object")

        # create the action client:
        self.movebase_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        # wait until the action server has started up and started listening for goals
        self.movebase_client.wait_for_server()

        # self.get_table()
        # current_table = rospy.get_param("/current_table")
        # self.move_to_table(current_table)

        person_found = False
        table_checked = False

        while (person_found == False) and (table_checked == False):

            # Moving to the next table
            status = self.get_table()
            if status == "table_checked":
                table_checked = True
                print("The person wasnt found at this table")
                self.interaction.talk("Sorry, but I couldnt find you" )
                return "outcome2"
            current_table = rospy.get_param("/current_table")
            self.move_to_table(current_table)

            self.interaction.talk("I am now going to look around until I can find you" )

            person_found = self.check_person_around_table()
            print person_found
            if person_found:
                return "outcome1"
        
        print("The person wasnt found at this table")
        self.interaction.talk("Sorry, but I couldnt find you" )
        
        return "outcome2"