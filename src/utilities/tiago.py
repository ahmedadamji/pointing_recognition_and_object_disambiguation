#!/usr/bin/python
import rospy
import actionlib
from rospy import ROSException, ROSInterruptException
import rosnode

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, PoseWithCovarianceStamped, Vector3
from nav_msgs.msg import OccupancyGrid

from pal_interaction_msgs.msg import TtsAction, TtsGoal


class Tiago:
    def __init__(self):

        self.move_base_goal_sent = False	# if goal is sent, we need to cancel before shutting down
        self.play_motion_goal_sent = False

        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.Rate(10)

        self.robot_pose = None
        self.snf_requests = []

		# set linear and angular velocities
        self.velocity = Twist()
		# publish and subcribe to relevant topics
        self.velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))
        rospy.loginfo("The move_base action server is up")

        self.play_motion = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        self.play_motion.wait_for_server(rospy.Duration(5))
        rospy.loginfo("The play_motion action server is up")

        self.tts_client = actionlib.SimpleActionClient('/tts', TtsAction)
        self.tts_client.wait_for_server(rospy.Duration(5))
        rospy.loginfo("The tts action server is up")

        self.point_head = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)
        self.point_head.wait_for_server(rospy.Duration(5))
        rospy.loginfo("The point_head action server is up")


    def move(self, linear=(0,0,0), angular=(0,0,0)):
        self.velocity.linear.x = linear[0] 	# Forward or Backward with in m/sec.
        self.velocity.linear.y = linear[1]
        self.velocity.linear.z = linear[2]

        self.velocity.angular.x = angular[0]
        self.velocity.angular.y = angular[1]
        self.velocity.angular.z = angular[2] 	# Anti-clockwise/clockwise in radians per sec

        self.velocity_publisher.publish(self.velocity)


    def play(self, motion_name, skip_planning=True):
        # send prescribed play motion goals
        self.play_motion_goal_sent = True
        play_goal = PlayMotionGoal()
        play_goal.motion_name = motion_name
        play_goal.skip_planning = skip_planning

        self.play_motion.send_goal_and_wait(play_goal)
        rospy.loginfo('Sent play_motion goal and waiting for robot to carry it out... ')



    def shutdown(self):
        if self.move_base_goal_sent:
            self.move_base.cancel_goal()
            rospy.loginfo("Stop Robot")
            rospy.sleep(1)

        if self.play_motion_goal_sent:
            self.play_motion.cancel_goal()
            rospy.loginfo("Stop Robot")
            rospy.sleep(1)
