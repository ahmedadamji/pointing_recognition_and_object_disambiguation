#!/usr/bin/python3.6

## ADD REFERENCE HERE

# Done this because python had problems importing the correct cv2
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

#importing mediapipe
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic

file_list = ['/tiago_ws/src/pointing_recognition/src/tests/media/image1.png', '/tiago_ws/src/pointing_recognition/src/tests/media/image2.jpeg', '/tiago_ws/src/pointing_recognition/src/tests/media/image3.jpeg',
             '/tiago_ws/src/pointing_recognition/src/tests/media/image4.jpeg', '/tiago_ws/src/pointing_recognition/src/tests/media/image5.jpeg',  '/tiago_ws/src/pointing_recognition/src/tests/media/image6.jpeg',
             '/tiago_ws/src/pointing_recognition/src/tests/media/image7.jpeg', '/tiago_ws/src/pointing_recognition/src/tests/media/image8.jpeg']

# For static images:
with mp_holistic.Holistic(static_image_mode=True) as holistic:
  for idx, file in enumerate(file_list):
    image = cv2.imread(file)
    image_height, image_width, _ = image.shape
    # Convert the BGR image to RGB before processing.
    results = holistic.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

    if results.pose_landmarks:
      print(
          f'Nose coordinates: ('
          f'{results.pose_landmarks.landmark[mp_holistic.PoseLandmark.NOSE].x * image_width}, '
          f'{results.pose_landmarks.landmark[mp_holistic.PoseLandmark.NOSE].y * image_height})'
      )
    # Draw pose, left and right hands, and face landmarks on the image.
    annotated_image = image.copy()
    mp_drawing.draw_landmarks(
        annotated_image, results.face_landmarks, mp_holistic.FACE_CONNECTIONS)
    mp_drawing.draw_landmarks(
        annotated_image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
    mp_drawing.draw_landmarks(
        annotated_image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
    # Use mp_holistic.UPPER_BODY_POSE_CONNECTIONS for drawing below when
    # upper_body_only is set to True.
    mp_drawing.draw_landmarks(
        annotated_image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS)
    cv2.imwrite('/tiago_ws/src/pointing_recognition/src/tests/media/mediapipe_annotated/annotated_image' + str(idx) + '.png', annotated_image)