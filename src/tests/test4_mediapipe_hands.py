#!/usr/bin/python3.6

# Done this because python had problems importing the correct cv2
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

#importing mediapipe
import mediapipe as mp

import numpy as np

## SOURCE CODE MODIFIED
## FROM: https://google.github.io/mediapipe/solutions/hands
## HAND CLASSIFICATION CODE: https://github.com/JuliaPoo/MultiHand-Tracking/blob/master/src/multi_hand_tracker.py

class ClassifyHands:
    def __init__(self):

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        self.classify_each_hand()

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
        
        leftHand = bool(self.get_hand_landmarks(hand_landmarks,5).x>self.get_hand_landmarks(hand_landmarks,17).x)
        rightHand = bool(self.get_hand_landmarks(hand_landmarks,5).x<self.get_hand_landmarks(hand_landmarks,17).x)
        print ("This is the left hand: ",leftHand)
        print ("This is the right hand: ",rightHand)

    def get_np_array_for_hand_landmarks(self,hand_landmarks,keypoint):
        return np.array([self.get_hand_landmarks(hand_landmarks,keypoint).x,self.get_hand_landmarks(hand_landmarks,keypoint).y,self.get_hand_landmarks(hand_landmarks,keypoint).z])


    def is_left_hand(self,hand_landmarks):
        # Returns True if hand_landmark is left hand and False if left hand.
        ## THIS WILL BE THE OTHER WAY ROUND IF THE CAMERA WAS NOT A SELFIE/WEB CAM AND WOULD INSTEAD BE TRUE FOR THE RIGHT HAND
        
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

    def classify_each_hand(self):

        # For webcam input:
        cap = cv2.VideoCapture(0)
        with self.mp_hands.Hands(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as hands:
            while cap.isOpened():
                success, image = cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    # If loading a video, use 'break' instead of 'continue'.
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

                        #self.classify_palm(hand_landmarks)
                        print(self.is_left_hand(hand_landmarks))

                        self.mp_drawing.draw_landmarks(
                            image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                cv2.imshow('MediaPipe Hands', image)
                if cv2.waitKey(5) & 0xFF == 27:
                    break
        cap.release()

def main(args):
    # Instantiating the class
    CH = ClassifyHands()

    # Ensure that the node continues running with rospy.spin()
    # Wrapping rospy.spin() in an exception handler in case of KeyboardInterrupts
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # To destroy all image windows before closing node
    cv2.destroyAllWindows()


# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
