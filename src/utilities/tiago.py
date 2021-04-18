#!/usr/bin/python
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

# Importing for speech recognition, interaction and replying to person
# import speech_recognition as sr
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import pyttsx
# For speech recognition
import speech_recognition as sr

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

    def get_data_from_speech(self, user_response):
        # Gathers user responses using speech

        recognizer = sr.Recognizer()
        microphone = sr.Microphone()
        # while loop to ensure voice is recognized
        while not user_response["success"] == True:
            with microphone as source:
                # adjusts the recognizer sensitivity to ambient noise
                recognizer.adjust_for_ambient_noise(source)
                # records audio from the microphone
                audio = recognizer.record(source, duration=4)
            
            try:
                # Recognizes speech recorded
                user_response["transcription"] = recognizer.recognize_google(audio).encode('ascii', 'ignore')
                user_response["success"] = True
                print user_response["transcription"]
            except sr.RequestError:
                # API was unreachable or unresponsive
                user_response["success"] = False
                user_response["error"] = "API unavailable"
            except sr.UnknownValueError:
                # speech was unintelligible
                user_response["success"] = False
                user_response["error"] = "Unable to recognize speech"

        return user_response


    def get_data_from_text(self, user_response):
        # Gathers user responses using text
        text = raw_input('Please type your response : ')
        user_response['transcription'] = text

        return user_response

    def get_data_from_user(self, request_type, valid_responses, type_of_data):

        response_valid = False
        while not response_valid:
        
            # dict to save the user response
            user_response = {
                "success": False,
                "error": None,
                "transcription": None
            }

            if request_type == "speech":
                user_response = self.get_data_from_speech(user_response)
            elif request_type == "text":
                user_response = self.get_data_from_text(user_response)

            if user_response['transcription'].lower() in valid_responses :
                response_valid = True

                return user_response['transcription']

            else:
                # if user response does not match the valid responses, error prompt to enter responses from possible options again
                print("Invalid entry")
                print("I am sorry, but " + user_response['transcription'] + " is not a valid entry for " + type_of_data)
                print("The valid responses for " + type_of_data + " are: ")
                print(valid_responses)
                print("\033[1;31;40m Please try again!  \n")

                # Reinforces to the user, the attribute collected
                self.talk("I am sorry, but " + user_response['transcription'] + " is not a valid entry for " + type_of_data)
                self.talk("The valid responses for " + type_of_data + " are: ")
                for item in valid_responses:
                    self.talk(item)
                self.talk("Please try again!")

    def get_start_command(self, request_type):

        response_valid = False
        while not response_valid:
        
            # dict to save the user response
            user_response = {
                "success": False,
                "error": None,
                "transcription": None
            }

            if request_type == "speech":
                user_response = self.get_data_from_speech(user_response)
            elif request_type == "text":
                user_response = self.get_data_from_text(user_response)

            return user_response['transcription']




    def shutdown(self):

        if self.play_motion_goal_sent:
            self.play_motion.cancel_goal()
            rospy.loginfo("Stop Robot")
            rospy.sleep(1)
