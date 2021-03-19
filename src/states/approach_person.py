#!/usr/bin/env python
import rospy
import actionlib


from utilities import Tiago
from smach import State
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
import tf


class ApproachPersonPointing(State):
    def __init__(self, classify):
        rospy.loginfo('ApproachPersonPointing state initialized')
        
        State.__init__(self, outcomes=['outcome1','outcome2'])
        
        #creates an instance of tiago class to interact with the user and perform physical actions
        self.tiago = Tiago()
        # Collects the details of tables in the environment from the util class and saves in self.tables
        self.tables = self.util.tables


    def get_table(self):
        for table_id in range(0, len(self.tables)):
            status = self.tables[table_id].get('status')
            if status == 'not checked':
                table_name = self.tables[table_id].get('name')
                print table_name + ' is the current table to be approached'
                rospy.set_param('/current_table', self.tables[table_id])
                return
        print("All tables have been checked")

    def detect_person(self):
        self.classify.subscribe_to_vision_messages()
        self.tiago.talk("I am now going to look around to see if i can find a person" )
        yolo_detections = self.classify.yolo_object_detection()
        # Not finding segmentations if no objects detected using yolo
        if not len(yolo_detections):
            return None
        self.classify.yolo_get_object_coordinates()

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
        quaternion = Quaternion(**orientation))
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        euler = (roll, pitch, yaw)

        # Setting the goal in Quaternion
        tiago_radians = euler[2]
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
        goal.target_pose.pose = Pose(position = Point(**location),
                                    orientation = Quaternion(**orientation))

        movebase_client.send_goal(goal)

        rospy.loginfo('GOAL SENT! o:')

        # waits for the server to finish performing the action
        if movebase_client.wait_for_result():
            rospy.loginfo('Goal location achieved!')
            # operator = getLocation()           
            # if operator:
            #     return get_closer_to_person(operator)
        else:
            rospy.logwarn("Couldn't reach the goal!")


    def execute(self, userdata, wait=True):
        rospy.loginfo('ApproachPersonPointing state executing')

        self.tiago.talk("I am now going to lift my torso, and then approach the person at table 0" )
        
        # Lift tiago's torso and set head to default
        self.tiago.lift_torso_head_default(True)

        # create the action client:
        movebase_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # wait until the action server has started up and started listening for goals
        movebase_client.wait_for_server()

        self.get_table()
        current_table rospy.get_param('/current_table')

        #location = rospy.get_param('/pointing_person_approach')
        location = current_table.get('person_check_location')

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose = Pose(position = Point(**location['position']),
                                    orientation = Quaternion(**location['orientation']))


        movebase_client.send_goal(goal)

        rospy.loginfo('GOAL SENT! o:')

        # waits for the server to finish performing the action
        if wait:
            if movebase_client.wait_for_result():

                rospy.set_param('/current_table/' + '/status', 'checked')
                rospy.loginfo('Goal location achieved!')
                self.tiago.talk("I have now reached the goal location" )
                # operator = getLocation()
                # if operator:
                #     return get_closer_to_person(operator)
            else:
                rospy.logwarn("Couldn't reach the goal!")
        
        if self.detect_person():
            print('Person was found at this table')
        else:
            self.rotate()
            print('No Person was found at this table')
        
        return 'outcome1'
