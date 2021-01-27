#!/usr/bin/env python

import rospy
import cv2
from lasr_object_detection_yolo.srv import YoloDetection
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError

class Classify:
    def __init__(self, dataset='coco'):
        # Defauts to coco dataset unless another trained model package is needed
        self.dataset = dataset
        self.yolo_detection = rospy.ServiceProxy('/yolo_detection', YoloDetection)
        self.bridge = CvBridge()

    def subscribe_to_vision_messages(self):

        rospy.loginfo('subscrbing to image messages')
        self.image_raw = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        self.points = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
    

    def yolo_object_detection(self):
        try:
            rospy.loginfo('waiting for yolo_detection service'))
            rospy.wait_for_service('/yolo_detection')
            rospy.loginfo('connected to yolo_detection service'))

            # running object recognition
            try:
                detect_objects = rospy.ServiceProxy('/yolo_detection', YoloDetection)
                detection_result = detect_objects(self.image_raw, self.dataset, 0.5, 0.3)
            except rospy.ServiceException as e:
                print('Object recognition failed')
                rospy.logwarn(e)
                return 'outcome1'


            frame = self.bridge.imgmsg_to_cv2(detection_result.image_bb, 'bgr8')
            cv2.imshow('frame', frame)
            cv2.waitKey(3)
            cv2.waitKey(3)
            try:
                frame = self.bridge.imgmsg_to_cv2(detection_result.image_bb, 'bgr8')
            except CvBridgeError as ex:
                rospy.logwarn(ex)
                return
            cv2.imshow('Detected Objects YOLO', frame)
            cv2.waitKey(5000)

            return detection_result.detected_objects

        except rospy.ROSInterruptException:
            pass
        except rospy.ServiceException as ex:
            rospy.logwarn('service call failed to {}'.format('/yolo_detection'))
            rospy.logwarn(ex)
        except rospy.ROSException as ex:
            rospy.logwarn('timed out waiting for {}'.format('/yolo_detection'))
            rospy.logwarn(ex)

    