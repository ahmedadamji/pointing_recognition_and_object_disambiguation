#!/usr/bin/env python
import rospy
import actionlib


from utilities import Tiago, Util
from smach import State
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
import tf
import math


class ApproachPersonPointing(State):
    def __init__(self, classify):
        rospy.loginfo('ApproachPersonPointing state initialized')
        
        State.__init__(self, outcomes=['outcome1','outcome2'])
        
        # creates an instance of classify class to classify yolo detections
        self.classify = classify
        #creates an instance of tiago class to interact with the user and perform physical actions
        self.tiago = Tiago()
        #creates an instance of util class to transform point frames
        self.util = Util()
        # Collects the details of tables in the environment from the util class and saves in self.tables
        self.tables = self.util.tables


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
        self.classify.subscribe_to_vision_messages()
        yolo_detections = self.classify.yolo_object_detection()
        # Not finding segmentations if no objects detected using yolo
        if not len(yolo_detections):
            return None
        #self.classify.yolo_get_object_coordinates()

        for index in range(len(yolo_detections)):
            if (yolo_detections[index].name == 'person'):
                print('A person was found in frame')
                return True
        print('No person was found in frame')
        return False

    def rotate_around_base(self, degrees):

        #getting current Tiago Location and Orientation in quaternion
        amcl_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        location = amcl_msg.pose.pose.position
        orientation = amcl_msg.pose.pose.orientation

        #converting to euler from quaternion pose
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        euler = (roll, pitch, yaw)

        # Setting the goal in Quaternion
        tiago_radians = euler[2]
        if tiago_radians < 0:
            tiago_radians += 2*math.pi
        # Finding target angle in radians
        target = degrees*(math.pi/180)
        goal_angle = tiago_radians+target
        location = Point(location.x, location.y, location.x)
        (x,y,z,w)= tf.transformations.quaternion_from_euler(0,0, goal_angle)
        orientation = Quaternion(x,y,z,w)


        # Sending move base goal
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position = location
        goal.target_pose.pose.orientation = orientation

        self.movebase_client.send_goal(goal)

        rospy.loginfo('GOAL SENT! o:')

        # waits for the server to finish performing the action
        if self.movebase_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
            # operator = getLocation()           
            # if operator:
            #     return get_closer_to_person(operator)
        else:
            rospy.logwarn("Couldn't reach the goal!")

    def move_to_table(self,current_table):
        #location = rospy.get_param('/pointing_person_approach')
        location = current_table.get('person_check_location')

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose = Pose(position = Point(**location['position']),
                                    orientation = Quaternion(**location['orientation']))


        self.movebase_client.send_goal(goal)

        rospy.loginfo('GOAL SENT! o:')

        # waits for the server to finish performing the action
        if self.movebase_client.wait_for_result():


            # rospy.set_param('/current_table/' + '/status', 'checked')
            rospy.loginfo('Goal location achieved!')
            self.tiago.talk("I have now reached the goal location" )
            # operator = getLocation()
            # if operator:
            #     return get_closer_to_person(operator)
        else:
            rospy.logwarn("Couldn't reach the goal!")

    def check_person_around_table(self):
        degrees = 0
        while degrees > -90:
            if not degrees == 0:
                self.rotate_around_base(degrees)
            if self.detect_person():
                print('Person was found at this table')
                return True
            degrees -= 45
                
        print('No Person was found at this table')
        return False



    def execute(self, userdata, wait=True):
        rospy.loginfo('ApproachPersonPointing state executing')

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
