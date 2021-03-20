#!/usr/bin/env python
import rospy
import actionlib

from smach import State
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
import tf
import math


class ApproachPerson(State):
    def __init__(self, classify_objects, tiago, util, move):
        rospy.loginfo('ApproachPerson state initialized')
        
        State.__init__(self, outcomes=['outcome1','outcome2'])
        
        # creates an instance of classify_objects class to classify yolo detections
        self.classify_objects = classify_objects
        #creates an instance of tiago class to interact with the user and perform physical actions
        self.tiago = tiago
        #creates an instance of util class to transform point frames
        self.util = util
        #creates an instance of move class to move robot across the map
        self.move = move


    def get_table(self):
        for table_id in range(0, len(self.tables)):
            status = self.tables[table_id].get('status')
            if status == 'not checked':
                table_name = self.tables[table_id].get('name')
                print table_name + ' is the current table to be approached'
                rospy.set_param('/current_table', self.tables[table_id])
                self.tables[table_id]["status"] = "checked"
                return
        print("All tables have been checked")
        return 'all_tables_checked'

    def detect_person(self):
        self.classify_objects.subscribe_to_vision_messages()
        yolo_detections = self.classify_objects.yolo_object_detection()
        # Not finding segmentations if no objects detected using yolo
        if not len(yolo_detections):
            return None
        #self.classify_objects.yolo_get_object_coordinates()

        for index in range(len(yolo_detections)):
            if (yolo_detections[index].name == 'person'):
                print('A person was found in frame')
                return True
        print('No person was found in frame')
        return False

    def move_to_table(self,current_table):
        #location = rospy.get_param('/pointing_person_approach')
        location = current_table.get('person_check_location')

        # Sending Move class the location to move to, and stores result in movebase
        movebase = self.move.move_base(location)
        if movebase == True:
            self.tiago.talk("I have now reached the goal location" )
        else:
            # INSERT HERE THE ACTION IF GOAL NOT ACHIEVED
            self.tiago.talk("I have not been able to reach the goal location" )


    def check_person_around_table(self):
        degrees = 0
        while degrees > -90:

            # Sending Move class the angle to rotate about, and stores result in rotate, and also returns the final movebase location
            rotate, location = self.move.rotate_around_base(degrees)
            if rotate == True:
                print("The robot has rotated around the base by " + str(degrees) + " degrees anticlockwise")
            else:
                # INSERT HERE THE ACTION IF GOAL NOT ACHIEVED
                print("The robot has not been able to rotate around it's base")

            if self.detect_person():
                print('Person was found at this table')
                # Saves the location where the pwerson was found to rosparm to use this later while looking back at person for gathering responses
                rospy.set_param('/pointing_person_approach', location)

                return True
            degrees -= 45
                
        print('No Person was found at this table')
        return False



    def execute(self, userdata, wait=True):
        rospy.loginfo('ApproachPerson state executing')

        # Collects the details of tables in the environment from the util class and saves in self.tables
        self.tables = self.util.tables

        # CHANGE TABLE NAME HERE:
        self.tiago.talk("I am now going to lift my torso, and then approach the person at table 0" )
        
        # Lift tiago's torso and set head to default
        self.tiago.lift_torso_head_default(True)

        # create the action client:
        self.movebase_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # wait until the action server has started up and started listening for goals
        self.movebase_client.wait_for_server()

        # self.get_table()
        # current_table = rospy.get_param('/current_table')
        # self.move_to_table(current_table)

        person_found = False
        all_tables_checked = False

        while (person_found == False) and (all_tables_checked == False):

            # Moving to the next table
            status = self.get_table()
            if status == 'all_tables_checked':
                all_tables_checked = True
            current_table = rospy.get_param('/current_table')
            self.move_to_table(current_table)

            self.tiago.talk("I am now going to look around to see if i can find a person" )

            person_found = self.check_person_around_table()
            print person_found
            if person_found:
                return 'outcome1'
        
        print('Person wasnt found at any table')
        
        return 'outcome2'
