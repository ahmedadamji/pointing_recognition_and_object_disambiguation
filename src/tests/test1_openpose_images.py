#!/usr/bin/python
import rospy
import cv2
import sys
sys.path.append('/tiago_ws/src/openpose/build/python')
from openpose import pyopenpose as op


def get_segmentation_from_image():
    try:
        # Set CV image
        im_path = '/tiago_ws/src/pointing_recognition/src/tests/media/image1.png'
        cv_image = cv2.imread(im_path)

        # Custom Params
        params = dict()
        params['model_folder'] = '/tiago_ws/src/openpose/models/'
        params['net_resolution'] = '320x176'
        params['hand'] = True
        params["face"] = True


        # Starting OpenPose
        #opWrapper = op.WrapperPython(op.ThreadManagerMode.Synchronous)
        # ^^ This makes the openpose segmentation visible via webcam.
        opWrapper = op.WrapperPython()
        opWrapper.configure(params)
        opWrapper.start()

        # Process Image
        datum = op.Datum()
        datum.cvInputData = cv_image
        opWrapper.emplaceAndPop(op.VectorDatum([datum]))


        # Display Image And Print Body Keypoints
        print("Body keypoints: \n" + str(datum.poseKeypoints))
        cv2.imshow("Image Window", datum.cvOutputData)
        cv2.waitKey(0)

    except Exception as e:
        print(e)
        sys.exit(-1)

def main(args):
    get_segmentation_from_image()
    rospy.init_node('hi')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)