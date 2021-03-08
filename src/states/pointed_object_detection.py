#!/usr/bin/env python
import rospy
import tf
import numpy as np
import cv2

from smach import State
from utilities import Tiago, Util

from geometry_msgs.msg import Point, Pose
from collections import namedtuple
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, RegionOfInterest, PointField
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped


# Refered catering_erl for yolo_object_recognition
class PointedObjectDetection(State):
    def __init__(self, classify):
        rospy.loginfo('PointedObjectDetection state initialized')
        
        State.__init__(self, outcomes=['outcome1','outcome2'])

        self.classify = classify
        self.bridge = CvBridge()

        #creates an instance of tiago class to interact with the user
        self.tiago = Tiago()
        #creates an instance of util class to transform point frames
        self.util = Util()
    

    def draw_bounding_box_around_intersection_point(self, intersection_point_world):
        
        print intersection_point_world
        camera_point_3d = self.util.transform_from_world_frame_to_camera_frame(intersection_point_world)
        print camera_point_3d
        self.camera_point_2d = self.util.get_2d_camera_point_from_3d_depth_point(camera_point_3d)
        print self.camera_point_2d

        image_raw = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        try:
            frame = self.bridge.imgmsg_to_cv2(image_raw, 'bgr8')
            # cv2.imshow('frame', frame)
        except CvBridgeError as ex:
            rospy.logwarn(ex)
            return
        box_start_point = (self.camera_point_2d[0]-150),(self.camera_point_2d[1]-150)
        box_end_point = self.camera_point_2d[0]+150,self.camera_point_2d[1]+150
        # Plotting the bounding box
        cv2.rectangle(frame, box_start_point, box_end_point, (0,0,255), 1)
        # Plotting the centre point of the bounding box
        cv2.circle(frame,(self.camera_point_2d[0],self.camera_point_2d[1]), 4, (0,150,150), 1)
        # Plots all figures on top of an opencv image of openpose keypoints
        cv2.imshow("Bounding Box For Pointed Objects", frame)
        cv2.waitKey(5000)

        return box_start_point, box_end_point
    
    def get_world_coordinate_for_current_object(self, current_yolo_detection):
        # This function is needed to get the world coordinate for each object to be used for disambiguation between objects in terms of location
        xywh = current_yolo_detection.xywh
        x = int (xywh[0] + (xywh[2]/2))
        y = int (xywh[1] + (xywh[3]/2))
        camera_point_3d = self.util.get_3d_depth_point_from_2d_camera_point([x,y])
        world_coordinate = self.util.transform_from_camera_frame_to_world_frame(camera_point_3d)
        return world_coordinate


    def detect_objects(self, box_start_point, box_end_point):

        self.classify.subscribe_to_vision_messages()
        yolo_detections = self.classify.yolo_object_detection(box_start_point, box_end_point, self.camera_point_2d)
        # Not finding segmentations if no objects detected using yolo
        total_objects_within_pointing_box = 0
        index_of_objects_inside_pointing_bounding_box = []
        objects_inside_bounding_box =[]
        if not len(yolo_detections):
            return None
        else:
            for i in range(len(yolo_detections)):
                xywh = yolo_detections[i].xywh
                if (((box_start_point[0] <= xywh[0] <= box_end_point[0]) and (box_start_point[1] <= xywh[1] <= box_end_point[1]))
                    and ((box_start_point[0] <= (xywh[0]+xywh[2]) <= box_end_point[0]) and (box_start_point[1] <= (xywh[1]+xywh[3]) <= box_end_point[1]))):
                    total_objects_within_pointing_box += 1
                    index_of_objects_inside_pointing_bounding_box.append(i)
        ## PARSING INTO NAME, CONFIDENCE AND COORDINATES OF DETECTION
        for o in range(len(index_of_objects_inside_pointing_bounding_box)):

            world_coordinate = self.get_world_coordinate_for_current_object(yolo_detections[index_of_objects_inside_pointing_bounding_box[o]])

            current_object = {
            "name": yolo_detections[index_of_objects_inside_pointing_bounding_box[o]].name,
            "confidence": yolo_detections[index_of_objects_inside_pointing_bounding_box[o]].confidence,
            "xywh": yolo_detections[index_of_objects_inside_pointing_bounding_box[o]].xywh,
            "world_coordinate": world_coordinate
            }
            objects_inside_bounding_box.append(current_object)
            # objects_inside_bounding_box.append([[yolo_detections[index_of_objects_inside_pointing_bounding_box[o]].name],
            #                                    [yolo_detections[index_of_objects_inside_pointing_bounding_box[o]].confidence],
            #                                    [yolo_detections[index_of_objects_inside_pointing_bounding_box[o]].xywh]])


        if total_objects_within_pointing_box == 0:
            print "No objects found within pointing bounding box"
        elif total_objects_within_pointing_box == 1:
            print "This is the only object being pointed at:"
            print yolo_detections[index_of_objects_inside_pointing_bounding_box[0]]
        else:
            print"Further diasambiguation needed"
        
        rospy.set_param('/objects_inside_bounding_box', objects_inside_bounding_box)
        rospy.set_param('/camera_point_after_object_detection_2d', [self.camera_point_2d[0], self.camera_point_2d[1]])

        self.classify.yolo_get_object_coordinates()
        


    def execute(self, userdata):
        rospy.loginfo('PointedObjectDetection state executing')

        intersection_point_world = rospy.get_param("/intersection_point_world")

        box_start_point, box_end_point = self.draw_bounding_box_around_intersection_point(intersection_point_world)

        self.detect_objects(box_start_point, box_end_point)



        # To destroy cv2 window at the end of state
        cv2.waitKey(0)
        #cv2.destroyAllWindows()
        
        return 'outcome1'
