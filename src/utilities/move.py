#!/usr/bin/python
# imported to ..................
import rospy
import actionlib

# imported to create move base goals
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
import math


class Move:

    def __init__(self):

        # Creating the Transform Listner:
        self.transformer = tf.TransformListener()

        # create the action client:
        self.movebase_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        # wait until the action server has started up and started listening for goals
        self.movebase_client.wait_for_server()

    def move_base(self, location):
        # Creating Target Pose Header:
        header = Header(frame_id = "map", stamp = rospy.Time.now())

        # creating Pose and MoveBaseGoal:
        pose = Pose(position = Point(**location["position"]), orientation = Quaternion(**location["orientation"]))

        # Storing the goal here:
        goal = MoveBaseGoal()
        goal.target_pose.header = header
        goal.target_pose.pose = pose

        # sending goal!
        self.movebase_client.send_goal(goal)

        # logging information about goal sent
        rospy.loginfo("GOAL SENT! o:")

        if self.movebase_client.wait_for_result():
            rospy.loginfo("Goal location achieved!")
            return True

        else:
            rospy.logwarn("Couldn't reach the goal!")
            return False

    def rotate_around_base(self, degrees):

        #getting current Tiago Location and Orientation in quaternion
        amcl_msg = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        position = amcl_msg.pose.pose.position
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
        # Adding current tiago angle to the target angle to rotate, to find goal anle to be achieved
        goal_angle = tiago_radians+target

        # Saving the Target Position and Orientation:
        position = Point(position.x, position.y, position.z)
        (x,y,z,w)= tf.transformations.quaternion_from_euler(0,0, goal_angle)
        orientation = Quaternion(x,y,z,w)


        # Sending move base goal
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position = position
        goal.target_pose.pose.orientation = orientation

        self.movebase_client.send_goal(goal)

        rospy.loginfo("GOAL SENT! o:")

        # Saving the Final location in a dict to pass back as the sucessful movebase location
        position_array = [position.x, position.y, position.z]
        orientation_array = [orientation.x, orientation.y, orientation.z, orientation.w]
        location = {
            "position": position_array,
            "orientation": orientation_array
        }
        # location = {
        #     "position": { "x": position.x, "y": position.y, "z": position.z },
        #     "orientation": { "x": orientation.x, "y": orientation.y, "z": orientation.z, "w": orientation.w }
        # }

        # waits for the server to finish performing the action
        if self.movebase_client.wait_for_result():
            rospy.loginfo("Goal location achieved!")
            return True, location
        else:
            rospy.logwarn("Couldn't reach the goal!")
            return False, location




