#!/usr/bin/python
import rospy
import actionlib

# Importing for speech recognition, interaction and replying to person
# import speech_recognition as sr
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import pyttsx
# For speech recognition
import speech_recognition as sr

class Interaction:
    def __init__(self):
        rospy.loginfo("Interaction Utility Initialised")

        # self.tts_client = actionlib.SimpleActionClient("/tts", TtsAction)
        # self.tts_client.wait_for_server(rospy.Duration(5))
        #rospy.loginfo("The tts action server is up")



    def talk(self, speech_in):
        # Create the TTS goal and send it
        print("\033[1;36mTIAGO: " + speech_in + "\033[0m")

        # init and set speech engine
        voiceEngine = pyttsx.init()
        voiceEngine.setProperty("rate", 150) # To reduce the speed of speech
        voiceEngine.say(speech_in)
        voiceEngine.runAndWait()
        
        # rate = voiceEngine.getProperty("rate")
        # volume = voiceEngine.getProperty("volume")
        # voice = voiceEngine.getProperty("voice")
        
        # print rate
        # print volume
        # print voice

        # Creating a TTS Goal (Text to speech) and sending it to tiago
        # tts_goal = TtsGoal()
        # tts_goal.rawtext.lang_id = "en_GB"
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
                audio = recognizer.record(source, duration=7)
            
            try:
                # Recognizes speech recorded
                user_response["transcription"] = recognizer.recognize_google(audio).encode("ascii", "ignore")
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
        text = raw_input("Please type your response : ")
        user_response["transcription"] = text

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
            
            words = user_response["transcription"].lower().split()
            for item in valid_responses:
                if (item in words[0:]):
                    response_valid = True
                    return item
                
            # if user response does not match the valid responses, error prompt to enter responses from possible options again
            print("Invalid entry")
            print("I am sorry, but " + user_response["transcription"] + " is not a valid entry for " + type_of_data)
            print("The valid responses for " + type_of_data + " are: ")
            print(valid_responses)
            print("\033[1;31;40m Please try again!  \n")

            # Reinforces to the user, the attribute collected
            self.talk("I am sorry, but " + user_response["transcription"] + " is not a valid entry for " + type_of_data)
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

            return user_response["transcription"]

