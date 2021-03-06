#!/usr/bin/env python
import rospy
import tf
import numpy as np
import cv2

from smach import State

from geometry_msgs.msg import Point, Pose
from collections import namedtuple
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, RegionOfInterest, PointField
from geometry_msgs.msg import Point, Pose, Quaternion, PointStamped, Vector3, PoseWithCovarianceStamped
import math


# Refered catering_erl for yolo_object_recognition
class PointedObjectDetection(State):
    def __init__(self, classify_objects, interaction, util):
        #rospy.loginfo("PointedObjectDetection state initialized")
        
        State.__init__(self, outcomes=["outcome1","outcome2"])

        self.classify_objects = classify_objects
        self.bridge = CvBridge()

        #creates an instance of interaction class to interact with the user
        self.interaction = interaction
        #creates an instance of util class to transform point frames
        self.util = util
    
    def define_bounding_region_around_intersection_point(self):
        self.radius_of_pointing = rospy.get_param("/radius_of_pointing")
        radius_x = self.radius_of_pointing*math.cos(math.pi/4)
        radius_y = self.radius_of_pointing*math.cos(math.pi/4)
        intersection_depth_coordinate = self.util.transform_from_world_frame_to_camera_frame(self.intersection_point_world)
        edge_of_bounding_area = self.util.get_2d_camera_point_from_3d_depth_point([intersection_depth_coordinate[0]+radius_x,intersection_depth_coordinate[1]+radius_y,intersection_depth_coordinate[2]])

        # sides = np.array([0.30,0.30,0.30])  # 30cm sides # Need to change this to a reasonable number
        # min_xyz = centre-(sides/2)
        # max_xyz = centre+(sides/2)

        # self.box_start_point_3d = min_xyz
        # self.box_end_point_3d = max_xyz


        # For visualisaition purposes on top of 2d image

        # TO FIND THE RADIUS IN PIXELS TO PLOT
        distance = np.array(np.array(edge_of_bounding_area)) -  np.array(self.intersection_point_2d)
        self.radius_of_pointing_2d = int(math.hypot(distance[0], distance[0]))

        # self.box_start_point_2d = self.util.get_2d_pixel_coordinate_from_world_coordinate(min_xyz)
        # self.box_end_point_2d = self.util.get_2d_pixel_coordinate_from_world_coordinate(max_xyz)

    

    def draw_bounding_region_around_intersection_point(self):
        
        #print self.intersection_point_world
        intersection_point_3d = self.util.transform_from_world_frame_to_camera_frame(self.intersection_point_world)
        #print intersection_point_3d
        self.intersection_point_2d = self.util.get_2d_camera_point_from_3d_depth_point(intersection_point_3d)
        # Finding the bounding box coordinates
        self.define_bounding_region_around_intersection_point()
        #print self.intersection_point_2d

        image_raw = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
        try:
            frame = self.bridge.imgmsg_to_cv2(image_raw, "bgr8")
            # cv2.imshow("frame", frame)
        except CvBridgeError as ex:
            rospy.logwarn(ex)
            return
        # box_start_point = (self.intersection_point_2d[0]-150),(self.intersection_point_2d[1]-150)
        # box_end_point = (self.intersection_point_2d[0]+150),(self.intersection_point_2d[1]+150)

        # Plotting the bounding box
        colour = (0,0,255)
        thickness = 1
        #cv2.rectangle(frame, self.box_start_point_2d, self.box_end_point_2d, (0,0,255), 1)

        # Plotting the centre point of the bounding box
        cv2.circle(frame, (self.intersection_point_2d[0],self.intersection_point_2d[1]), self.radius_of_pointing_2d, colour, thickness)
        # Plots all figures on top of an opencv image of openpose keypoints
        cv2.imshow("Bounding Box For Pointed Objects", frame)
        cv2.waitKey(5000)

    
    def get_world_coordinate_for_object(self, yolo_detection):
        # This function is needed to get the world coordinate for each object to be used for disambiguation between objects in terms of location
        xywh = yolo_detection.xywh
        x = int (xywh[0] + (xywh[2]/2))
        y = int (xywh[1] + (xywh[3]/2))
        camera_point = [x,y]
        world_coordinate = self.util.get_world_coordinate_from_2d_pixel_coordinate(camera_point)
        return world_coordinate


    def detect_objects(self):

        self.classify_objects.subscribe_to_vision_messages()
        yolo_detections = self.classify_objects.yolo_object_detection(self.radius_of_pointing_2d, self.intersection_point_2d)
        # Not finding segmentations if no objects detected using yolo
        if not len(yolo_detections):
            return None
        # self.classify_objects.yolo_get_object_coordinates()
        return yolo_detections

    def get_object_indices(self, yolo_detections):

        objects_within_pointing_bounding_region_indices = []

        for i in range(len(yolo_detections)):
            #print yolo_detections[i]
            # Finding world coordinate for each detection to check weather it is within pointing bounding box and on the table
            world_coordinate = self.get_world_coordinate_for_object(yolo_detections[i])
            # # need intersection_point_world as centre of bounding box
            #self.intersection_point_world


            # xywh = yolo_detections[i].xywh

            # if (((self.box_start_point_3d[0] <= xywh[0] <= self.box_end_point_3d[0]) and (self.box_start_point_3d[1] <= xywh[1] <= self.box_end_point_3d[1]))
            #     and ((self.box_start_point_3d[0] <= (xywh[0]+xywh[2]) <= self.box_end_point_3d[0]) and (self.box_start_point_3d[1] <= (xywh[1]+xywh[3]) <= self.box_end_point_3d[1]))):

            # A point lies inside the sphere if:
            #( point.x-centre.x ) ^2 + (point.y-centre.y) ^2 + (point.z-centre.z) ^ 2 <= r^2

            dx_sq = math.pow((world_coordinate[0]-self.intersection_point_world[0]), 2)
            dy_sq = math.pow((world_coordinate[1]-self.intersection_point_world[1]), 2)
            dz_sq = math.pow((world_coordinate[2]-self.intersection_point_world[2]), 2)
            distance_between_point_and_centre = dx_sq + dy_sq + dz_sq
            rad_sq = math.pow(self.radius_of_pointing, 2)

            if (distance_between_point_and_centre <= rad_sq):
                self.total_objects_within_pointing_bounding_region += 1
                objects_within_pointing_bounding_region_indices.append(i)

                
            # if ((self.box_start_point_3d[0] <= world_coordinate[0] <= self.box_end_point_3d[0]) and (self.box_start_point_3d[1] <= world_coordinate[1] <= self.box_end_point_3d[1]) 
            #     and (self.box_start_point_3d[2] <= world_coordinate[2] <= self.box_end_point_3d[2])):

            #     self.total_objects_within_pointing_bounding_region += 1
            #     objects_within_pointing_bounding_region_indices.append(i)
        
        return objects_within_pointing_bounding_region_indices




    def get_objects_within_pointing_bounding_region(self, yolo_detections):

        self.total_objects_within_pointing_bounding_region = 0
        self.objects_within_pointing_bounding_region =[]

        self.objects_within_pointing_bounding_region_indices = self.get_object_indices(yolo_detections)



        ## PARSING INTO NAME, CONFIDENCE AND COORDINATES OF DETECTION
        for index in self.objects_within_pointing_bounding_region_indices:

            world_coordinate = self.get_world_coordinate_for_object(yolo_detections[index])
            ## not classifying the table as an object
            if (not yolo_detections[index].name == "diningtable") and (not yolo_detections[index].name == "person"):

                current_object = {
                "name": yolo_detections[index].name.lower(),
                "confidence": yolo_detections[index].confidence,
                "xywh": yolo_detections[index].xywh,
                "world_coordinate": [world_coordinate[0].item(), world_coordinate[1].item(), world_coordinate[2].item()]
                }
                self.objects_within_pointing_bounding_region.append(current_object)
            else:
                self.total_objects_within_pointing_bounding_region -= 1
            
            
        # Checking if any objects found are capable of disambiguation
        # if found object equals one, this does not need to be disambiguated.
        if len(self.objects_within_pointing_bounding_region) > 1:
            count = 0
            for current_object in self.objects_within_pointing_bounding_region:
                # IF - ELSE block to ensure only objects capable of disambiguation are used for this state
                if current_object.get("name") in self.util.list_of_objects_capable_of_disambiguation:
                    count +=1
            if count == 0:
                return False
            else:
                return True
        else:
            return True


    def notify_all_objects(self):
        self.interaction.talk("I found " + str(self.total_objects_within_pointing_bounding_region) + " objects close to where you were pointing, which were ")

        previous_object_name = self.objects_within_pointing_bounding_region[0].get("name")
        count = 1
        for i in range(len(self.objects_within_pointing_bounding_region)-1):
            object_name = self.objects_within_pointing_bounding_region[i+1].get("name")
            if previous_object_name == object_name:
                count += 1
            previous_object_name = object_name
            
        if count == len(self.objects_within_pointing_bounding_region):
            self.interaction.talk("All " + previous_object_name + "s.")
            return

        objects_found = ''
        for i in range(len(self.objects_within_pointing_bounding_region)):
            if i == (len(self.objects_within_pointing_bounding_region)-1):
                objects_found += " and "
            object_name = self.objects_within_pointing_bounding_region[i].get("name")
            objects_found += object_name
            if not ((i == (len(self.objects_within_pointing_bounding_region)-1)) or (i == (len(self.objects_within_pointing_bounding_region)-2))):
                objects_found += ", "

        self.interaction.talk(objects_found)
        return

    def execute(self, userdata):
        rospy.loginfo("PointedObjectDetection state executing")

        self.intersection_point_world = rospy.get_param("/intersection_point_world")
        self.current_table = rospy.get_param("/current_table")

        self.interaction.talk("The objects I found close to where you were pointing are displayed within a circle in the image shown to you now" )
        self.draw_bounding_region_around_intersection_point()

        yolo_detections = self.detect_objects()

        capable_of_disambiguation = self.get_objects_within_pointing_bounding_region(yolo_detections)

        if self.total_objects_within_pointing_bounding_region == 0:
            #print "No objects were found within pointing bounding box"
            self.interaction.talk("Sorry but I couldn't find any objects close to where you were pointing")

            return "outcome2"

        elif self.total_objects_within_pointing_bounding_region == 1:
            #print "This is the only object being pointed at:"
            self.interaction.talk("The object you asked me to identify was a " + self.objects_within_pointing_bounding_region[0].get("name"))
            #print yolo_detections[self.objects_within_pointing_bounding_region_indices[0]]

            return "outcome2"
        
        if not capable_of_disambiguation:
            #print "Objects found are not capable of disambiguation"
            self.notify_all_objects()
            self.interaction.talk("unfortunately, I am unable to determine exactly which one of these you were referring to.")

            return "outcome2"


        else:
            self.notify_all_objects()

            #print"Further diasambiguation needed"
            self.interaction.talk("I will now ask you some simple questions, to determine exactly which of the " + str(self.total_objects_within_pointing_bounding_region) + " objects is being pointed.")
        
        #rospy.set_param("/objects_on_table", objects_on_table)
        rospy.set_param("/objects_within_pointing_bounding_region", self.objects_within_pointing_bounding_region)  # CHECK IF THE FOLLOWING IS NEEDED.
        rospy.set_param("/camera_point_after_object_detection_2d", [self.intersection_point_2d[0], self.intersection_point_2d[1]])



        # To destroy cv2 window at the end of state
        #cv2.destroyAllWindows()
        
        return "outcome1"
