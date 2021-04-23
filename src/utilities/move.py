#!/usr/bin/python
# imported to ..................
import rospy
import actionlib

# imported to create move base goals
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
import tf
import math


class Move:

    def __init__(self):
        rospy.loginfo("Move Utility Initialised")

        # Creating the Transform Listner:
        self.transformer = tf.TransformListener()

        # create the action client:
        self.movebase_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        # wait until the action server has started up and started listening for goals
        self.movebase_client.wait_for_server()
        rospy.loginfo("The move_base action server is up")

        # cancel previously sent goal before shutting down rospy
        self.play_motion_goal_sent = False
        # shutting down rospy
        rospy.on_shutdown(self.shutdown)


        # Starting playmotion action server:
        self.play_motion = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)
        self.play_motion.wait_for_server(rospy.Duration(5))
        rospy.loginfo("The play_motion action server is up")

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


    def lift_torso_head_default(self, wait=False):
        # lift torso height and head to default
        self.play_motion_goal_sent = True

        # Uncomment this if sending torso goal errors out again:
        # torso_goal = FollowJointTrajectoryGoal()

        # retrieveing play motion goal from motions.yaml
        pm_goal = PlayMotionGoal("back_to_default", True, 0)

        test_goal = PlayMotionGoal()
        #print test_goal.priority

        # Sending play motion goal
        self.play_motion.send_goal(pm_goal)
        
        if wait:
            self.play_motion.wait_for_result()

        print("play motion: back_to_default completed")


    def check_table(self, wait=False):
        # lift torso height and head look down
        self.play_motion_goal_sent = True

        # Uncomment this if sending torso goal errors out again:
        # torso_goal = FollowJointTrajectoryGoal()

        # retrieveing play motion goal from motions.yaml
        pm_goal = PlayMotionGoal("check_table", True, 0)

        test_goal = PlayMotionGoal()
        #print test_goal.priority

        # Sending play motion goal
        self.play_motion.send_goal(pm_goal)

        if wait:
            self.play_motion.wait_for_result()

        print("play motion: check_table completed")

    def look_at_person(self, wait=False):
        # lift torso height and head to default
        self.play_motion_goal_sent = True

        # Uncomment this if sending torso goal errors out again:
        # torso_goal = FollowJointTrajectoryGoal()

        # retrieveing play motion goal from motions.yaml
        pm_goal = PlayMotionGoal("look_at_person", True, 0)

        test_goal = PlayMotionGoal()
        #print test_goal.priority

        # Sending play motion goal
        self.play_motion.send_goal(pm_goal)
        
        if wait:
            self.play_motion.wait_for_result()



    def shutdown(self):

        if self.play_motion_goal_sent:
            self.play_motion.cancel_goal()
            rospy.loginfo("Stop Robot")
            rospy.sleep(1)




