#!/usr/bin/python3.6

import rospy
# Done this because python had problems importing the correct cv2
import sys
ros_path = "/opt/ros/kinetic/lib/python2.7/dist-packages"
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")

#importing mediapipe
import mediapipe as mp

import numpy as np

from pointing_recognition.srv import HandClassification, HandClassificationResponse

## SOURCE CODE MODIFIED
## FROM: https://google.github.io/mediapipe/solutions/hands
## HAND CLASSIFICATION CODE: https://github.com/JuliaPoo/MultiHand-Tracking/blob/master/src/multi_hand_tracker.py

class classify_hands_server:
    def __init__(self):
        rospy.init_node("classify_hands")

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands

        serv = rospy.Service("classify_hands", HandClassification, self.classify_each_hand)
        rospy.loginfo("classify_hands service initialised")
        rospy.spin()

    def get_hand_landmarks(self,hand_landmarks,keypoint):
        if keypoint == 0:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        if keypoint == 1:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_CMC]
        if keypoint == 2:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_MCP]
        if keypoint == 3:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP]
        if keypoint == 4:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        if keypoint == 5:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
        if keypoint == 6:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP]
        if keypoint == 7:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_DIP]
        if keypoint == 8:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        if keypoint == 9:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
        if keypoint == 10:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
        if keypoint == 11:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_DIP]
        if keypoint == 12:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        if keypoint == 13:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_MCP]
        if keypoint == 14:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_PIP]
        if keypoint == 15:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_DIP]
        if keypoint == 16:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        if keypoint == 17:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP]
        if keypoint == 18:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_PIP]
        if keypoint == 19:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_DIP]
        if keypoint == 20:
            return hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]
    

    
    def classify_palm(self,hand_landmarks):
        ## THIS WILL BE THE OTHER WAY ROUND IF THE CAMERA WAS NOT A SELFIE/WEB CAM AND WOULD INSTEAD BE TRUE FOR THE RIGHT HAND
        
        # If 1 does not work try 5
        leftHand = bool(self.get_hand_landmarks(hand_landmarks,1).x>self.get_hand_landmarks(hand_landmarks,17).x)
        rightHand = bool(self.get_hand_landmarks(hand_landmarks,1).x<self.get_hand_landmarks(hand_landmarks,17).x)
        print ("This is the left hand: ",leftHand)
        print ("This is the right hand: ",rightHand)

    def get_np_array_for_hand_landmarks(self,hand_landmarks,keypoint):
        return np.array([self.get_hand_landmarks(hand_landmarks,keypoint).x,self.get_hand_landmarks(hand_landmarks,keypoint).y,self.get_hand_landmarks(hand_landmarks,keypoint).z])


    def is_left_hand(self,hand_landmarks):
        # Returns True if hand_landmark is left hand and False if left hand.
        ## THIS WILL BE THE OTHER WAY ROUND IF THE CAMERA WAS NOT A SELFIE/WEB CAM AND WOULD INSTEAD BE TRUE FOR THE RIGHT HAND
        ## HAND CLASSIFICATION CODE: https://github.com/JuliaPoo/MultiHand-Tracking/blob/master/src/multi_hand_tracker.py
        
        digitgroups = [
            (17,18,19,20),
            (13,14,15,16),
            (9,10,11,12),
            (5,6,7,8),
            (2,3,4) # Thumb
        ]
        
        palm_dir_vec = np.array([0,0,0], dtype=np.float64)
        for digit in digitgroups:
            for idx in digit[1:]:
                palm_dir_vec += self.get_np_array_for_hand_landmarks(hand_landmarks,idx) - self.get_np_array_for_hand_landmarks(hand_landmarks,digit[0])
              
        palm_pos_vec = np.array([0,0,0], dtype=np.float64)
        for digit in digitgroups:
            palm_pos_vec += self.get_np_array_for_hand_landmarks(hand_landmarks,digit[0])  
        palm_pos_vec /= len(digitgroups)
        
        top_palm_pos_vec = self.get_np_array_for_hand_landmarks(hand_landmarks,9)
        
        val = np.dot(np.cross(self.get_np_array_for_hand_landmarks(hand_landmarks,2) - palm_pos_vec, palm_dir_vec), top_palm_pos_vec - palm_pos_vec)

        if val < 0: return True
        
        return False

    def classify_each_hand(self, req):
        self.right_hand_counter = 0
        self.left_hand_counter = 0

        # For webcam input:
        cap = cv2.VideoCapture(0)
        with self.mp_hands.Hands(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as hands:
            while cap.isOpened():
                success, image = cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    # If loading a video, use "break" instead of "continue".
                    continue

                # Flip the image horizontally for a later selfie-view display, and convert
                # the BGR image to RGB.
                image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                image.flags.writeable = False
                results = hands.process(image)

                # Draw the hand annotations on the image.
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                if results.multi_hand_landmarks:

                    for hand_landmarks in results.multi_hand_landmarks:

                        self.mp_drawing.draw_landmarks(
                            image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                        #self.classify_palm(hand_landmarks)
                        classification = self.is_left_hand(hand_landmarks)
                        if classification == False:
                            self.right_hand_counter += 1
                        elif classification == True:
                            self.left_hand_counter += 1

                cv2.imshow("MediaPipe Hands", image)
                if self.right_hand_counter>30:
                    return HandClassificationResponse("right")
                elif self.left_hand_counter>30:
                    return HandClassificationResponse("left")
                if cv2.waitKey(5) & 0xFF == 27:
                    break
        cap.release()

if __name__ == "__main__":
    classify_hands_server()
