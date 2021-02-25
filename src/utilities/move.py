#!/usr/bin/python
# imported to ..................
import rospy
import actionlib

# imported to create move base goals
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move():
    header = Header(frame_id = 'map', stamp = rospy.Time.now())

    # creating Pose and MoveBaseGoal:
    
    pose = Pose()
    pose.position.x = 2.5
    pose.position.y = -2.2
    pose.position.z = 0
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 1
    pose.orientation.w = 3
    
    # Storing the goal here:
    goal = MoveBaseGoal()
    goal.target_pose.header = header
    goal.target_pose.pose = pose
    
    # creating the action client:
    move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    move_base_client.wait_for_server()


    # sending goal!
    move_base_client.send_goal(goal)

    # logging information about goal sent
    rospy.loginfo('GOAL SENT! o:')
    move_base_client.wait_for_result()
    #move_base_client.get_result()
    
    # logging information about goal reached
    rospy.loginfo('GOAL REACHED! (:')


if __name__ == '__main__':
    rospy.init_node('move')
    try:
        move()
    except rospy.ROSInterruptException:
        pass
