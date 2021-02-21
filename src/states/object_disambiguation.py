#!/usr/bin/env python
import rospy
from smach import State
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from sensor_msgs.srv import SetCameraInfo
import sensor_msgs.point_cloud2 as pc2

from pointing_recognition.msg import IntersectionData

from geometry_msgs.msg import PoseStamped

import cv2
import math
import numpy as np



class ObjectDisambiguation(State):
    def __init__(self):
        rospy.loginfo('ObjectDisambiguation state initialized')
        State.__init__(self, outcomes=['outcome1','outcome2'])
        
    
    def execute(self, userdata):
        rospy.loginfo('ObjectDisambiguation state executing')
        

        
        except Exception as e:
            print(e)
            sys.exit(-1)

        return 'outcome1'
