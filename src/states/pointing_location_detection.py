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


    def angle_between_points( self, a, b, c ):
        rospy.loginfo('Calculating angle between points')
        ba = np.array(a) - np.array(b)
        bc = np.array(c) - np.array(b)

        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle = np.arccos(cosine_angle)

        print np.degrees(angle)
        return np.degrees(angle)

    def get_elbow_angle(self, shoulder,elbow,wrist, hand):
        #To correctly calculate the angles, this must be done using the 3D points found at these keypoints and therefore this function needs to move to yje pointing_location_detection.py file
        rospy.loginfo('Calculating elbow angle')
        angle = 0
        angle = self.angle_between_points(shoulder, elbow, wrist)
        rospy.loginfo('%s angle:%f'%(hand,angle))
        return angle

    def get_wrist_chest_delta(self, wrist, spine_chest, hand):
        # After testing, change hand tip delta as a test parameter for is pointing or not to height difference between hand and chest
        rospy.loginfo('Calculating hand tip and chest delta')
        ## IN THE REPORT PUT A DIAGRAM OF THE CORDINATE FRAME OF THE DEPTH CAMERA OF TIAGO AND JUSTIFY THIS POINT
        #comparing the height of chest and hand tip, and as the person will be standing up in the y axis of the camera frame, this will give the height diffrence between the two points
        # Because the coordinate frames for y for the camera are positive downward, the heigh diffrence is calculated as the negative value
        wrist_chest_delta = -(wrist[1] - spine_chest[1])
        print("The wrist_chest_delta for the "+ str(hand) + " hand is " + str(wrist_chest_delta) + " meters")
        #rospy.loginfo('%s wrist_chest_delta:%f'%(hand, wrist_chest_delta))
        return wrist_chest_delta

    def get_wrist_shoulder_delta(self, wrist, shoulder, hand):
        # This parameter is needed to make the classifications for pointing invalid if a person is pointing above a certain height of the shoulder,
        # as the application does not require pointing up towards a wall, standing objects or the ceiling
        
        rospy.loginfo('Calculating hand tip and shoulder delta')
        ## IN THE REPORT PUT A DIAGRAM OF THE CORDINATE FRAME OF THE DEPTH CAMERA OF TIAGO AND JUSTIFY THIS POINT
        #comparing the height of chest and hand tip, and as the person will be standing up in the y axis of the camera frame, this will give the height diffrence between the two points
        # Because the coordinate frames for y for the camera are positive downward, the heigh diffrence is calculated as the negative value
        wrist_shoulder_delta = -(wrist[1] - shoulder[1])
        print("The wrist_shoulder_delta for the "+ str(hand) + " hand is " + str(wrist_shoulder_delta) + " meters")
        #rospy.loginfo('%s wrist_shoulder_delta:%f'%(hand, wrist_shoulder_delta))
        return wrist_shoulder_delta

    def check_finger_tip(self, hand_tip, hand):
        if (math.isnan(hand_tip[0])) or (math.isnan(hand_tip[1])) or (math.isnan(hand_tip[2])):
            print("Sorry, but I could not detect the exact location of the person's "+ hand + " finger tip to be used for pointing location detection")
            self.tiago.talk("Sorry, but I could not detect the exact location of the person's "+ hand + " finger tip to be used for pointing location detection")
            return "isnan"

    def is_pointing(self):

        right_shoulder = np.array(self.pose_keypoints.right_shoulder)
        left_shoulder = np.array(self.pose_keypoints.left_shoulder)
        left_elbow = np.array(self.pose_keypoints.left_elbow)
        right_elbow = np.array(self.pose_keypoints.right_elbow)
        spine_chest = np.array(self.pose_keypoints.spine_chest)
        left_hand_tip = np.array(self.pose_keypoints.left_hand_tip)
        right_hand_tip = np.array(self.pose_keypoints.right_hand_tip)
        left_wrist = np.array(self.pose_keypoints.left_wrist)
        right_wrist = np.array(self.pose_keypoints.right_wrist)

        print right_shoulder,right_elbow,right_wrist,'right'
        print left_shoulder,left_elbow,left_wrist,'left'
        left_elbow_angle = self.get_elbow_angle(left_shoulder,left_elbow,left_wrist,'left')
        right_elbow_angle = self.get_elbow_angle(right_shoulder,right_elbow,right_wrist,'right')
        left_wrist_chest_delta = self.get_wrist_chest_delta(left_wrist,spine_chest,'left')
        right_wrist_chest_delta = self.get_wrist_chest_delta(right_wrist,spine_chest,'right')
        left_wrist_shoulder_delta = self.get_wrist_shoulder_delta(left_wrist,left_shoulder,'left')
        right_wrist_shoulder_delta = self.get_wrist_shoulder_delta(right_wrist,right_shoulder,'right')


        # Parameters that need to be satisfied in case hand is pointing based on observed data points
        # Later try to shift this functionality to is_pointing() funtion

        ## Reduced value from -0.1 for hand tip delta from microsoft PSI's implementation as the position of the chest is a rough estimate in comparison to the original keypoint
        # The value used is the best case value when person is closest to table
        # Additionaly the value they have used is not based upon pointing downwards, and therefore an extra distance between the hand tip and the chest is needed (verify this)
        # An additional parameter is used here for wrist_shoulder_delta, which ensures person pointing above a certain height is not considered.

        if ((left_elbow_angle > 120)and(left_wrist_chest_delta>-0.40)and(left_wrist_shoulder_delta<0)):
            print('left hand has been raised in pointing position')
            self.tiago.talk("I can see that the person's left hand has been raised in pointing position")

            hand = 'left'

            if(self.check_finger_tip(left_hand_tip, hand) == "isnan"):
                return

            return hand, left_hand_tip

            
        elif ((right_elbow_angle > 120)and(right_wrist_chest_delta>-0.40)and(right_wrist_shoulder_delta<0)):
            print('right hand has been raised in pointing position')
            self.tiago.talk("I can see that the person's right hand has been raised in pointing position")

            hand = 'right'

            if(self.check_finger_tip(right_hand_tip, hand) == "isnan"):
                return

            return hand, right_hand_tip


        else:
            print('hand not pointing')
            hand = 'none_pointing'

            return hand, [0,0,0]


    def normalize(self, array):
        #rospy.loginfo('normalizing recieved array')
        normalized = array/np.linalg.norm(array, axis = 0)
        return normalized

    def get_pointing_line(self, hand_tip, head, open_pose_output_image):
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

        return start_point_3d, end_point_3d, start_point_2d, end_point_2d


    # Move this function to util
    def project_depth_array_to_2d_image_pixels(self, point_3d):
        #rospy.loginfo('projecting depth array to 2d image pixels')
        camera_info = rospy.wait_for_message('/xtion/rgb/camera_info', CameraInfo)
        depth_array = np.array([point_3d[0], point_3d[1], point_3d[2], 1])
        uvw = np.dot(np.array(camera_info.P).reshape((3, 4)), depth_array.transpose()).transpose()
        x = int(uvw[0] / uvw[2])
        y = int(uvw[1] / uvw[2])

        return x,y

    def intersect_line_with_depth_mesh(self, start_point, end_point, maxDistance = 5, skipFactor = 0.05):
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

            # Making sure the hypothesis point does not go outside the camera frame
            if (hypothesis_point_2d[0] >= 640) or (hypothesis_point_2d[1] >= 420) or (hypothesis_point_2d[0] <= 0) or (hypothesis_point_2d[1] <= 0):
                print ("Couldn't find an intersection with the depth mesh within the camera frame")
                # POTENTIAL FUTURE IMPROVEMENT --> Move the robot head along the pointing line so that it is not limited by the camera frame
                return
            
            # get the mesh distance of the hypothesis point by checking the depth data of the 2D coordinate:
            meshDistance = self.xyz_array[hypothesis_point_2d[0]][hypothesis_point_2d[1]][2]

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

        print ("Couldn't find an intersection with the depth mesh within the maximum distance for pointing")
        return
        

    ## ATTEMPTS AT FINDING COLLISION WITH MESH -->

            # exit = False
            # for x in range(640 - tip[0] - 20):
            #     for y in range(480 - tip[1] - 20):
            #         if math.isnan((self.xyz_array[x+tip[0]+20][y+tip[1]+20])[0]):
            #             point = Point3D(0,0,0)
            #         else:
            #             recorded_point = np.array(self.xyz_array[x+tip[0]+20][y+tip[1]+20])
            #             point = Point3D(recorded_point[0],recorded_point[1],recorded_point[2])
                    
            #         intersection = np.array(line.intersection(point))
            #         if intersection.size > 0:
            #             print(line.intersection(point))
            #             exit = True
            #         if(exit):
            #             break
            #     if(exit):
            #         break


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



    def display_pointing_line(self, open_pose_output_image,start_point_2d, intersection_point_2d):
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

    def set_params(self, intersection_point_2d, intersection_point_3d, head):
        
        # self.msg_to_send.intersection_point_2d = intersection_point_2d
        # self.msg_to_send.intersection_point_3d = intersection_point_3d

        # self.intersection_point_pub.publish(self.msg_to_send)

        rospy.set_param('/intersection_point_2d', [intersection_point_2d[0].item(), intersection_point_2d[1].item()])
        rospy.set_param('/intersection_point_3d', [intersection_point_3d[0].item(), intersection_point_3d[1].item(), intersection_point_3d[2].item()])
        intersection_point_world = self.util.transform_from_camera_frame_to_world_frame(intersection_point_3d)
        rospy.set_param('/intersection_point_world', [intersection_point_world[0].item(), intersection_point_world[1].item(), intersection_point_world[2].item()])
        #rospy.set_param('/start_point_3d', [start_point_3d[0].item(), start_point_3d[1].item(), start_point_3d[2].item()])

        #cv2.waitKey(0)


        # Saving world coordinate for head for use during disambiguation in reference to user location
        person_head_world_coordinate = self.util.transform_from_camera_frame_to_world_frame(head)
        rospy.set_param('/person_head_world_coordinate', [person_head_world_coordinate[0].item(), person_head_world_coordinate[1].item(), person_head_world_coordinate[2].item()])

    def detect_pointing_location(self, hand_tip):
        open_pose_output_image = self.bridge.imgmsg_to_cv2(self.pose_keypoints.open_pose_output_image_msg, "bgr8")
        head = np.array(self.pose_keypoints.head)

        start_point_3d, end_point_3d, start_point_2d, end_point_2d  = self.get_pointing_line(hand_tip, head, open_pose_output_image)

        maxDistance = 5
        skipFactor = 0.05

        try:
            intersection_point_3d, intersection_point_2d = self.intersect_line_with_depth_mesh(start_point_3d, end_point_3d, maxDistance, skipFactor)

            self.display_pointing_line(open_pose_output_image,start_point_2d, intersection_point_2d)

            self.set_params(intersection_point_2d, intersection_point_3d, head)

            return True

        except Exception as e:
            print('Couldnt detect pointing location')
            rospy.logwarn(e)
            return False


    def execute(self, userdata):
        rospy.loginfo('PointingLocationDetection state executing')

        self.tiago.talk("I am now going to look towards the person and determine weather he / she is pointing, and at which location" )


        img_msg = rospy.wait_for_message('/xtion/rgb/image_raw',Image)
        
        # To save the depth coordinates once so that I don't have to call it again and doesnt slow everything
        ## MAKE XYZ_ARRAY A GLOBAL VARIABLE
        depth_points = rospy.wait_for_message('/xtion/depth_registered/points',PointCloud2)
        self.xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(depth_points, remove_nans=False)
        self.xyz_array = np.transpose(self.xyz_array, (1, 0, 2))

        try:
            rospy.loginfo('waiting for openpose_detection service')
            rospy.wait_for_service('/openpose_detection')
            rospy.loginfo('connected to openpose_detection service')

            # running openpose detection
            try:
                openpose_detection = rospy.ServiceProxy('/openpose_detection', OpenPoseKeypoints)
                self.pose_keypoints = openpose_detection(img_msg, depth_points)

            except rospy.ServiceException as e:
                print('Pointing detection failed')
                rospy.logwarn(e)
                return 'outcome2'

            try:
                hand, hand_tip = self.is_pointing()
            except Exception as e:
                print('Couldnt detect pointing location')
                rospy.logwarn(e)
                return 'outcome2'

            if hand == 'none_pointing':
                self.tiago.talk("I can see that the person is not pointing at any table")
                return 'outcome2'

            self.tiago.talk("I see that the person is pointing with their " + str(hand) + " hand at a table")

            result = self.detect_pointing_location(hand_tip)

            if result == False:
                self.tiago.talk("I cannot find the location for where the person is pointing")
                return 'outcome2'

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
