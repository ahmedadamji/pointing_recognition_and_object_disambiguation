#!/usr/bin/env python
import rospy

from utilities import Tiago
from smach import State
from geometry_msgs.msg import Point, Pose
from collections import namedtuple

# Refered catering_erl for yolo_object_recognition
class ObjectDetection(State):
    def __init__(self, classify):
        rospy.loginfo('ObjectDetection state initialized')
        
        State.__init__(self, outcomes=['outcome1','outcome2'])

        # creates an instance of classify class to classify yolo detections
        self.classify = classify
        # creates an instance of tiago class to interact with the user and perform physical actions
        self.tiago = Tiago()

    def detect_objects(self):

        self.classify.subscribe_to_vision_messages()
        self.tiago.talk("I am now going to look straight ahead and classify objects using yolo object detection" )
        yolo_detections = self.classify.yolo_object_detection()
        # Not finding segmentations if no objects detected using yolo
        if not len(yolo_detections):
            return None
        self.classify.yolo_get_object_coordinates()
        


    def execute(self, userdata):
        rospy.loginfo('ObjectDetection state executing')

        self.detect_objects()

        #opWrapper.stop()
        return 'outcome1'
