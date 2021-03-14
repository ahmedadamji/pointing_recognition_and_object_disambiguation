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

class ClassifyHands:
    def __init__(self):

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        self.classify_each_hand()

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

                        ## THIS WILL BE THE OTHER WAY ROUND IF THE CAMERA WAS NOT A SELFIE/WEB CAM AND WOULD INSTEAD BE TRUE FOR THE RIGHT HAND
                        leftHand = bool(hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].x>hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP].x)
                        rightHand = bool(hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].x<hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP].x)
                        print ("This is the left hand: ",leftHand)
                        print ("This is the right hand: ",rightHand)

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
