#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, PointCloud2, CameraInfo

from pointing_recognition.msg import IntersectionData


from smach import State

from pointing_recognition.srv import OpenPoseKeypoints

import cv2
import math
import numpy as np
import ros_numpy

# import sympy
# from sympy import Point2D, Point3D
# from sympy.abc import L
# from sympy.geometry import Line2D, Line3D, Segment3D


# cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

class PointingLocationDetection(State):
    def __init__(self, tiago, util):
        rospy.loginfo('PointingLocationDetection state initialized')
        State.__init__(self, outcomes=['outcome1','outcome2'])
        
        self.intersection_point_pub = rospy.Publisher('/intersection_point', IntersectionData)
        self.msg_to_send = IntersectionData()
        
        self.bridge = CvBridge()
        
        #creates an instance of tiago class to interact with the user
        self.tiago = tiago
        #creates an instance of util class to transform point frames
        self.util = util


    def normalize(self, array):
        #rospy.loginfo('normalizing recieved array')
        normalized = array/np.linalg.norm(array, axis = 0)
        return normalized

    # def GetMeshDepthAtPoint(self, ICameraIntrinsics depthIntrinsics, depth_points, Point3D point, bool undistort):
    #     Point2D depthSpacePoint = self.ToPixelSpace(point, undistort)

    #     x = int (math.round(depthSpacePoint.x))
    #     y = int (math.round(depthSpacePoint.y))
    #     if ((x < 0) or (x >= depth_points.width) or (y < 0) or (y >= depth_points.height)):
    #         return float('NaN')

    #     byteOffset = int ((y * depth_points.stride) + (x * 2))
    #     depth = int(depth_points.ReadBytes(2, byteOffset))
    #     if (depth == 0):
    #         return float('NaN')

    #     return float (depth / 1000)

    # def ToPixelSpace(self, Point3D pt, bool distort):
    #     # X points in the depth dimension. Y points to the left, and Z points up.
    #     pixelPt = Point2D((-pt.y / pt.x), (-pt.z / pt.x))
    #     if (distort):
    #         this.DistortPoint(pixelPt, out pixelPt)

    #     tmp = Point3D(pixelPt.x, pixelPt.y, 1.0)
    #     tmp = tmp.TransformBy(this.transform)
    #     return Point2D(tmp.x, tmp.y)


    # Move this function to util
    def project_depth_array_to_2d_image_pixels(self, point_3d):
        #rospy.loginfo('projecting depth array to 2d image pixels')
        camera_info = rospy.wait_for_message('/xtion/rgb/camera_info', CameraInfo)
        depth_array = np.array([point_3d[0], point_3d[1], point_3d[2], 1])
        uvw = np.dot(np.array(camera_info.P).reshape((3, 4)), depth_array.transpose()).transpose()
        x = int(uvw[0] / uvw[2])
        y = int(uvw[1] / uvw[2])

        return x,y

    def intersect_line_with_depth_mesh(self, xyz_array, skipFactor, maxDistance, start_point, end_point):
        # SECOND ATTEMPT AT FINDING COLLISION WITH MESH -->
        delta = skipFactor * self.normalize(end_point - start_point)
        maxSteps = int(maxDistance / (np.linalg.norm(delta)))
        hypothesis_point_3d = start_point
        for i in range(maxSteps):
            # The delta is the maximum feature size in the x axis of the depth camera frame of the object,
            # the hypothesis point may not lie on an object appearing smaller than this size from the distance of the robot.
            # This may also affect objects that are curved on the edges and if the sides do not lie within the pointing line.
            hypothesis_point_3d += delta
            # Get 2D xy coordinate of the hypothesis point
            hypothesis_point_2d = np.array(self.project_depth_array_to_2d_image_pixels(hypothesis_point_3d))
            print("Advancing across line of pointing")
            # get the mesh distance of the hypothesis point by checking the depth data of the 2D coordinate:
            meshDistance = xyz_array[hypothesis_point_2d[0]][hypothesis_point_2d[1]][2]

            # if the mesh distance is less than the distance to the point we've hit the mesh
            # can do so that in the selected pixel space, I can compare the depth, if the
            # depth of the pointcloud is less then it is intersecting
            # or i can just check if the point is inside the box selected
            # ADD ANOTHER CONDITION OF hypothesis_point_3d[2]- meshDistance SHOULD BE LESS THAN A CERTAIN AMOUNT
            if (not(math.isnan(meshDistance)) and (meshDistance < hypothesis_point_3d[2])):
                hypothesis_point_3d = [hypothesis_point_3d[0], hypothesis_point_3d[1], hypothesis_point_3d[2]]

                print "The location of pointing is identified at:"
                print "DEPTH COORDINATES: "
                print hypothesis_point_3d
                print "RGB IMAGE COORDINATES: "
                print hypothesis_point_2d

                return hypothesis_point_3d, hypothesis_point_2d

                ## TEST HERE IF THERE IS OBSTRUCTION TO POINTING LINE< WILL IT GIVE ERROR< ALSO MAKE IT SUCH THAT THE INTERSECTION IS ONLY CHECKED WITHIN A RADIUS
                ## OF THE POINTING LINE TO ELIMINATE PROBLEMS WITH FAR AWAY OBJECTS OVERLAPPING WITH MESH, CHECK PERFORMANCE OF THIS AS WELL AND INCLUDE RESULTS OF BOTH
                ## BEFORE AND AFTER IN REPORT
        

        ## FIRST ATTEMPTH AT FINDING COLLISION WITH MESH -->
        # exit = False
        # for x in range(640 - tip[0] - 20):
        #     for y in range(480 - tip[1] - 20):
        #         if math.isnan((xyz_array[x+tip[0]+20][y+tip[1]+20])[0]):
        #             point = Point3D(0,0,0)
        #         else:
        #             recorded_point = np.array(xyz_array[x+tip[0]+20][y+tip[1]+20])
        #             point = Point3D(recorded_point[0],recorded_point[1],recorded_point[2])
                
        #         intersection = np.array(line.intersection(point))
        #         if intersection.size > 0:
        #             print(line.intersection(point))
        #             exit = True
        #         if(exit):
        #             break
        #     if(exit):
        #         break



    def get_pointing_line(self, hand_tip, head, xyz_array, open_pose_output_image, maxDistance = 5, skipFactor = 0.05):
        rospy.loginfo('calucating the line of pointing')
        
        # https://github.com/mikedh/trimesh/blob/master/examples/ray.py
        # https://github.com/mikedh/trimesh/issues/211
        # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        # https://github.com/microsoft/psi
        # https://github.com/microsoft/psi/blob/master/Sources/Calibration/Microsoft.Psi.Calibration/CalibrationExtensions.cs
        # https://github.com/microsoft/psi/blob/dd60dbbc2651ed2a515b9d0b1c2ea2d2adfc2e25/Sources/Calibration/Microsoft.Psi.Calibration/CameraIntrinsics.cs#L101

        # find a way to get the depth mesh from the rgbd camera

        # Finding tip location to determine which area of image to detect for overlap
        #tip = [int(hand[8][0]), int(hand[8][1])]

        direction = self.normalize(hand_tip - head)
        # print(np.shape(direction))

        # Do not start the line right at the hand-tip to avoid having an intersection with the mesh around the hand.
        start_point = hand_tip + (direction*0.05)
        end_point = hand_tip + (direction*0.15)

        start_point_3d = np.array(start_point)
        end_point_3d = np.array(end_point)

        # line = Line3D(start_point_3d, end_point_3d)


        start_point_2d = np.array(self.project_depth_array_to_2d_image_pixels(start_point_3d))
        end_point_2d = np.array(self.project_depth_array_to_2d_image_pixels(end_point_3d))

        # print(start_point_3d, start_point_2d, end_point_3d, end_point_2d)

        # rospy.loginfo('displaying pointing line')
        # cv2.line(open_pose_output_image, (start_point_2d[0],start_point_2d[1]), (end_point_2d[0],end_point_2d[1]), (0,0,255), 2)
        # cv2.imshow("Pointing Line Results", open_pose_output_image)
        # cv2.waitKey(5000)
        # comment this later to stop state machine automatically and move past this code
        
        #cv2.waitKey(0)

        intersection_point_3d, intersection_point_2d = self.intersect_line_with_depth_mesh(xyz_array, skipFactor, maxDistance, start_point, end_point)
        
        rospy.loginfo('displaying extended pointing line towards mesh')
        self.tiago.talk("I have computed the line of pointing and I am now going to display it to you" )
        # Stores extended line upto mesh
        cv2.line(open_pose_output_image, (start_point_2d[0],start_point_2d[1]), (intersection_point_2d[0],intersection_point_2d[1]), (255,255,0), 1)
        # Stores box around overlapping point
        box_start_point = (intersection_point_2d[0]-25),(intersection_point_2d[1]-25)
        box_end_point = intersection_point_2d[0]+25,intersection_point_2d[1]+25
        cv2.rectangle(open_pose_output_image, box_start_point, box_end_point, (0,0,255), 1)
        # Plots all figures on top of an opencv image of openpose keypoints
        cv2.imshow("Pointing Line Results", open_pose_output_image)
        cv2.waitKey(5000)
        
        # self.msg_to_send.intersection_point_2d = intersection_point_2d
        # self.msg_to_send.intersection_point_3d = intersection_point_3d

        # self.intersection_point_pub.publish(self.msg_to_send)

        rospy.set_param('/intersection_point_2d', [intersection_point_2d[0].item(), intersection_point_2d[1].item()])
        rospy.set_param('/intersection_point_3d', [intersection_point_3d[0].item(), intersection_point_3d[1].item(), intersection_point_3d[2].item()])
        intersection_point_world = self.util.transform_from_camera_frame_to_world_frame(intersection_point_3d)
        rospy.set_param('/intersection_point_world', [intersection_point_world[0].item(), intersection_point_world[1].item(), intersection_point_world[2].item()])
        #rospy.set_param('/start_point_3d', [start_point_3d[0].item(), start_point_3d[1].item(), start_point_3d[2].item()])

        #cv2.waitKey(0)




    def execute(self, userdata):
        rospy.loginfo('PointingLocationDetection state executing')

        self.tiago.talk("I am now going to look towards the person and determine weather he / she is pointing, and at which location" )
        # # Flags
        # args = self.set_flags()

        # # Set Params
        # params = self.set_params()

        img_msg = rospy.wait_for_message('/xtion/rgb/image_raw',Image)
        
        # To save the depth coordinates once so that I don't have to call it again and doesnt slow everything
        ## MAKE XYZ_ARRAY A GLOBAL VARIABLE
        depth_points = rospy.wait_for_message('/xtion/depth_registered/points',PointCloud2)
        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(depth_points, remove_nans=False)
        xyz_array = np.transpose(xyz_array, (1, 0, 2))

        try:
            rospy.loginfo('waiting for openpose_detection service')
            rospy.wait_for_service('/openpose_detection')
            rospy.loginfo('connected to openpose_detection service')

            # running object recognition
            try:
                compute_pointing = rospy.ServiceProxy('/openpose_detection', OpenPoseKeypoints)
                self.pointing_result = compute_pointing(img_msg, depth_points)

            except rospy.ServiceException as e:
                print('Pointing detection failed')
                rospy.logwarn(e)
                return 'outcome2'
            
            self.tiago.talk("I see that the person is pointing with their " + str(self.pointing_result.hand) + " hand")
            open_pose_output_image = self.bridge.imgmsg_to_cv2(self.pointing_result.open_pose_output_image_msg, "bgr8")
            hand_tip = np.array(self.pointing_result.hand_tip)
            head = np.array(self.pointing_result.head)
            self.get_pointing_line(hand_tip, head, xyz_array, open_pose_output_image, 1, 0.05) # (hand_tip, head, open_pose_output_image, maxDistance, skipFactor)

            # Saving world coordinate for head for use during disambiguation in reference to user location
            person_head_world_coordinate = self.util.transform_from_camera_frame_to_world_frame(self.pointing_result.head)
            rospy.set_param('/person_head_world_coordinate', [person_head_world_coordinate[0].item(), person_head_world_coordinate[1].item(), person_head_world_coordinate[2].item()])

            return 'outcome1'
            

        except rospy.ROSInterruptException:
            pass
        except rospy.ServiceException as ex:
            rospy.logwarn('service call openpose_detection failed')
            rospy.logwarn(ex)
        except rospy.ROSException as ex:
            rospy.logwarn('timed out waiting for openpose_detection service')
            rospy.logwarn(ex)


        
        # To destroy cv2 window at the end of state
        cv2.destroyAllWindows()

        #self.opWrapper.stop()
        
        return 'outcome1'
