#!/usr/bin/env python

import rospy
import cv2
import tf
import numpy as np
from lasr_object_detection_yolo.srv import YoloDetection
#from jeff_segment_objects.srv import SegmentObjects, SegmentObjectsRequest
from sensor_msgs.msg import Image, PointCloud2, RegionOfInterest
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
from cv_bridge import CvBridge, CvBridgeError

class ClassifyObjects:
    def __init__(self, dataset="coco"):
        rospy.loginfo("ClassifyObjects Utility Initialised")
        # Defauts to coco dataset unless another trained model package is needed
        self.dataset = dataset
        self.yolo_detection = rospy.ServiceProxy("/yolo_detection", YoloDetection)
        #self.segment_objects_srv = rospy.ServiceProxy("/segment_objects", SegmentObjects)
        self.bridge = CvBridge()

    def subscribe_to_vision_messages(self):

        print("subscrbing to image messages")
        self.image_raw = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
        self.points = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
    
    # Box start point and endpoint are set to defaut 0,0 as they are only needed in some states when the data is passed.
    def yolo_object_detection(self, radius_of_pointing_2d = 0, camera_point_2d = (0,0)):
        try:
            rospy.loginfo("waiting for yolo_detection service")
            rospy.wait_for_service("/yolo_detection")
            rospy.loginfo("connected to yolo_detection service")

            # running object recognition
            try:
                detect_objects = rospy.ServiceProxy("/yolo_detection", YoloDetection)
                self.detection_result = detect_objects(self.image_raw, self.dataset, 0.1, 0.3) # (image, dataset, confidence, nms)
                print("YOLO detection results : ", self.detection_result.detected_objects)
            except rospy.ServiceException as e:
                print("Object recognition failed")
                rospy.logwarn(e)
                return


            try:
                frame = self.bridge.imgmsg_to_cv2(self.detection_result.image_bb, "bgr8")
            except CvBridgeError as ex:
                rospy.logwarn(ex)
                # To destroy cv2 window at the end of state
                cv2.destroyAllWindows()
                return

            # Plotting the bounding box
            colour = (0,0,255)
            thickness = 1
            #cv2.rectangle(frame, box_start_point, box_end_point, (0,0,255), 1)
            #cv2.circle(frame, (camera_point_2d[0],camera_point_2d[1]), radius_of_pointing_2d, colour, thickness)
            # Plotting the centre point of the bounding box
            cv2.circle(frame, (camera_point_2d[0],camera_point_2d[1]), radius_of_pointing_2d, colour, thickness)


            # Plots all figures on top of an opencv image of openpose keypoints
            cv2.imshow("YOLO Object Detection Results", frame)
            cv2.waitKey(5000)

            # To destroy cv2 window at the end of state
            cv2.destroyAllWindows()
            return self.detection_result.detected_objects

        except rospy.ROSInterruptException:
            pass
        except rospy.ServiceException as ex:
            rospy.logwarn("service call yolo_detection failed")
            rospy.logwarn(ex)
        except rospy.ROSException as ex:
            rospy.logwarn("timed out waiting for yolo_detection service")
            rospy.logwarn(ex)



    # def yolo_get_object_coordinates(self):
    #     # segments objects in pcl by clustering
    #     # npr is about how much of the cloud to filter out
    #     # cluster_tol distance between each point to be considered in 1 cluster. 0.02 is 2cm
    #     # last two are min and max number of points in a cluster

    #     try:
    #         rospy.loginfo("waiting for segment_objects server")
    #         rospy.wait_for_service("/segment_objects")
    #         rospy.loginfo("connected to segment_objects server")
            
    #         segmentation_request = SegmentObjectsRequest(self.points, 0.5, 0.02)
    #         segmentation_result = self.segment_objects_srv(segmentation_request)

    #         width = self.points.width

    #         print("No of clusters found from segment objects : ")
    #         print(len(segmentation_result.clusters))

    #         bounding_boxes = []
    #         for cluster in segmentation_result.clusters:
    #             left = cluster.indices[0]%width
    #             right = cluster.indices[0]%width
    #             top = int(cluster.indices[0]/width)
    #             bottom = int(cluster.indices[0]/width)

    #             for index in cluster.indices:
    #                 row = int(index/width)
    #                 column = index%width

    #                 left = min(left, column)
    #                 right = max(right, column)
    #                 top = min(top, row)
    #                 bottom = max(bottom, row)

    #             bounding_box = RegionOfInterest()
    #             bounding_box.x_offset = left
    #             bounding_box.y_offset = top
    #             bounding_box.width = right - left
    #             bounding_box.height = bottom - top
    #             bounding_boxes.append(bounding_box)


    #         try:
    #             frame = self.bridge.imgmsg_to_cv2(self.image_raw, "bgr8").copy()
    #             # cv2.imshow("frame", frame)
    #         except CvBridgeError as ex:
    #             rospy.logwarn(ex)
    #             return
    #         for bb in bounding_boxes:
    #             cv2.rectangle(frame, (bb.x_offset, bb.y_offset),
    #                 (bb.x_offset + bb.width, bb.y_offset + bb.height), (255, 0, 255), 1)

    #         cv2.imshow("PCL Object Segmentation Results", frame)
    #         cv2.waitKey(5000)

    #         # To destroy cv2 window at the end of state
    #         # cv2.destroyAllWindows()

    #     except rospy.ServiceException as ex:
    #         rospy.logwarn(ex)