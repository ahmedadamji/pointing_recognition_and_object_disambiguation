#!/usr/bin/python
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

# Importing for speech recognition, interaction and replying to person
# import speech_recognition as sr
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import pyttsx

class Tiago:
    def __init__(self):

        # cancel previously sent goal before shutting down rospy
        self.play_motion_goal_sent = False
        # shutting down rospy
        rospy.on_shutdown(self.shutdown)

        # Uncomment this if sending torso goal errors out again:
        # self.enable_torso = actionlib.SimpleActionClient("/torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)

        # Starting playmotion action server:
        self.play_motion = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)
        self.play_motion.wait_for_server(rospy.Duration(5))
        rospy.loginfo("The play_motion action server is up")

        # self.play_motion_client = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)
        # self.play_motion_client.wait_for_server(rospy.Duration(5))
        # rospy.loginfo("The /play_motion action server is up")

        self.tts_client = actionlib.SimpleActionClient('/tts', TtsAction)
        self.tts_client.wait_for_server(rospy.Duration(5))
        rospy.loginfo("The tts action server is up")

        self.load_object_features()


    def lift_torso_head_default(self, wait=False):
        # lift torso height and head to default
        self.play_motion_goal_sent = True

        # Uncomment this if sending torso goal errors out again:
        # torso_goal = FollowJointTrajectoryGoal()

        # retrieveing play motion goal from motions.yaml
        pm_goal = PlayMotionGoal('back_to_default', True, 0)

        test_goal = PlayMotionGoal()
        print test_goal.priority

        # Sending play motion goal
        self.play_motion.send_goal(pm_goal)
        
        if wait:
            self.play_motion.wait_for_result()

        rospy.loginfo("play motion: back_to_default completed")


    def check_table(self, wait=False):
        # lift torso height and head look down
        self.play_motion_goal_sent = True

        # Uncomment this if sending torso goal errors out again:
        # torso_goal = FollowJointTrajectoryGoal()

        # retrieveing play motion goal from motions.yaml
        pm_goal = PlayMotionGoal('check_table', True, 0)

        test_goal = PlayMotionGoal()
        print test_goal.priority

        # Sending play motion goal
        self.play_motion.send_goal(pm_goal)

        if wait:
            self.play_motion.wait_for_result()

        rospy.loginfo("play motion: check_table completed")

    def look_at_person(self, wait=False):
        # lift torso height and head to default
        self.play_motion_goal_sent = True

        # Uncomment this if sending torso goal errors out again:
        # torso_goal = FollowJointTrajectoryGoal()

        # retrieveing play motion goal from motions.yaml
        pm_goal = PlayMotionGoal('look_at_person', True, 0)

        test_goal = PlayMotionGoal()
        print test_goal.priority

        # Sending play motion goal
        self.play_motion.send_goal(pm_goal)
        
        if wait:
            self.play_motion.wait_for_result()

        rospy.loginfo("play motion: back_to_default completed")

    def talk(self, speech_in):
        # Create the TTS goal and send it
        print('\033[1;36mTIAGO: ' + speech_in + '\033[0m')

        # init and set speech engine
        voiceEngine = pyttsx.init()
        voiceEngine.setProperty('rate', 150) # To reduce the speed of speech
        voiceEngine.say(speech_in)
        voiceEngine.runAndWait()
        
        # rate = voiceEngine.getProperty('rate')
        # volume = voiceEngine.getProperty('volume')
        # voice = voiceEngine.getProperty('voice')
        
        # print rate
        # print volume
        # print voice

        # Creating a TTS Goal (Text to speech) and sending it to tiago
        # tts_goal = TtsGoal()
        # tts_goal.rawtext.lang_id = 'en_GB'
        # tts_goal.rawtext.text = speech_in
        # self.tts_client.send_goal(tts_goal)

    def load_object_features(self):

        object_attributes_parm = rospy.get_param('/object_attributes')
        self.object_attributes = [
            {
                'name':    objects['name'],
                'colour':  objects['colour'],
                'type':    objects['type'],
                'texture': objects['texture'],
                'size':    objects['size'],
                'shape':   objects['shape']
            }
            for objects in object_attributes_parm
        ]


    def shutdown(self):

        if self.play_motion_goal_sent:
            self.play_motion.cancel_goal()
            rospy.loginfo("Stop Robot")
            rospy.sleep(1)
