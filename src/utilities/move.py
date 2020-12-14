#!/usr/bin/python
import rospy
import actionlib

# import these to create move base goals
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move():
    header = Header(frame_id = 'map', stamp = rospy.Time.now())

    # create the Pose and MoveBaseGoal:
    
    pose = Pose()
    pose.position.x = 1
    pose.position.y = 1
    pose.position.z = 0
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 1
    pose.orientation.w = 0
    
    # Fill in the goal here
    goal = MoveBaseGoal()
    goal.target_pose.header = header
    goal.target_pose.pose = pose
    
    # create the action client:
    
    move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    move_base_client.wait_for_server()


    # send the goal!
    move_base_client.send_goal(goal)
    # print on goal sent
    rospy.loginfo('GOAL SENT! o:')
    move_base_client.wait_for_result()
    #move_base_client.get_result()
    
    # print on finish
    rospy.loginfo('GOAL REACHED! (:')


if __name__ == '__main__':
    rospy.init_node('lasr_move_tutorial')
    try:
        move()
    except rospy.ROSInterruptException:
        pass
