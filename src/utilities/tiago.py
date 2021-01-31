#!/usr/bin/python
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


class Tiago:
    def __init__(self):

        # if goal is sent, we need to cancel before shutting down
        self.play_motion_goal_sent = False

        rospy.on_shutdown(self.shutdown)
        self.enable_torso = actionlib.SimpleActionClient("/torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.play_motion = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)
        self.play_motion.wait_for_server(rospy.Duration(5))
        rospy.loginfo("The play_motion action server is up")

        # self.play_motion_client = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)
        # self.play_motion_client.wait_for_server(rospy.Duration(5))
        # rospy.loginfo("The /play_motion action server is up")


    def lift_torso_head_default(self, wait=False):
        # lift torso high and head to default
        self.play_motion_goal_sent = True
        torso_goal = FollowJointTrajectoryGoal()
        pm_goal = PlayMotionGoal('back_to_default', True, 0)
        test_goal = PlayMotionGoal()
        print test_goal.priority
        #self.play_motion.send_goal(torso_goal)
        self.play_motion.send_goal(pm_goal)

        if wait:
            self.play_motion.wait_for_result()

        rospy.loginfo("play motion: back_to_default")


    def shutdown(self):

        if self.play_motion_goal_sent:
            self.play_motion.cancel_goal()
            rospy.loginfo("Stop Robot")
            rospy.sleep(1)
