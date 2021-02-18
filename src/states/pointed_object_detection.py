#!/usr/bin/env python
import rospy
import tf
import numpy as np
import cv2

from smach import State
from geometry_msgs.msg import Point, Pose
from collections import namedtuple
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, PointCloud2, RegionOfInterest, PointField, CameraInfo
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped


# Refered catering_erl for yolo_object_recognition
class PointedObjectDetection(State):
    def __init__(self, classify):
        rospy.loginfo('PointedObjectDetection state initialized')
        
        State.__init__(self, outcomes=['outcome1','outcome2'])

        self.classify = classify
        self.bridge = CvBridge()
        self.transformer = tf.TransformListener()
    
    def transform_from_world_frame_to_camera_frame(self, world_point):
        # http://wiki.ros.org/tf
        # http://docs.ros.org/en/indigo/api/tf/html/c++/classtf_1_1Transformer.html

        self.map_points = rospy.wait_for_message('/map', OccupancyGrid)

        self.transformer.waitForTransform('map', 'xtion_rgb_optical_frame', self.map_points.header.stamp, rospy.Duration(2.0))

        intersection_point_world = PointStamped()
        intersection_point_world.header = self.map_points.header
        intersection_point_world.point = Point(*world_point)

        person_point = self.transformer.transformPoint('xtion_rgb_optical_frame', intersection_point_world)
        #print person_point
        tf_point = person_point.point
        camera_point_3d = np.array([tf_point.x, tf_point.y, tf_point.z])
        return camera_point_3d
    
    def project_depth_array_to_2d_image_pixels(self, point_3d):
        rospy.loginfo('projecting depth array to 2d image pixels')
        camera_info = rospy.wait_for_message('/xtion/rgb/camera_info', CameraInfo)
        depth_array = np.array([point_3d[0], point_3d[1], point_3d[2], 1])
        uvw = np.dot(np.array(camera_info.P).reshape((3, 4)), depth_array.transpose()).transpose()
        x = int(uvw[0] / uvw[2])
        y = int(uvw[1] / uvw[2])

        return x,y

    def draw_bounding_box_around_intersection_point(self, intersection_point_world):
        
        print intersection_point_world
        camera_point_3d = self.transform_from_world_frame_to_camera_frame(intersection_point_world)
        print camera_point_3d
        camera_point_2d = np.array(self.project_depth_array_to_2d_image_pixels(camera_point_3d))
        print camera_point_2d

        image_raw = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        try:
            frame = self.bridge.imgmsg_to_cv2(image_raw, 'bgr8')
            # cv2.imshow('frame', frame)
        except CvBridgeError as ex:
            rospy.logwarn(ex)
            return
        box_start_point = (camera_point_2d[0]-100),(camera_point_2d[1]-100)
        box_end_point = camera_point_2d[0]+100,camera_point_2d[1]+100
        cv2.rectangle(frame, box_start_point, box_end_point, (0,0,255), 1)
        # Plots all figures on top of an opencv image of openpose keypoints
        cv2.imshow("Bounding Box For Pointed Objects", frame)
        cv2.waitKey(5000)

        return box_start_point, box_end_point


    def detect_objects(self, box_start_point, box_end_point):

        self.classify.subscribe_to_vision_messages()
        yolo_detections = self.classify.yolo_object_detection(box_start_point, box_end_point)
        # Not finding segmentations if no objects detected using yolo
        if not len(yolo_detections):
            return None
        elif len(yolo_detections) == 1:
            print "This is the only object being pointed at"
        else:
            print"Further disambiguation needed"
        self.classify.yolo_get_object_coordinates()
        


    def execute(self, userdata):
        rospy.loginfo('PointedObjectDetection state executing')

        intersection_point_world = rospy.get_param("/intersection_point_world")

        box_start_point, box_end_point = self.draw_bounding_box_around_intersection_point(intersection_point_world)

        self.detect_objects(box_start_point, box_end_point)

        #opWrapper.stop()
        return 'outcome1'
