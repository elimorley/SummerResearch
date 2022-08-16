import rospy
import numpy as np
import cv2
import sys
from utils import ARUCO_DICT
import argparse
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os


def pose_esitmation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters)

        # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                       distortion_coefficients)
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 

    return frame, rvec, tvec

bridge = CvBridge()
#count = 0

def image_callback(img_msg):
    rospy.loginfo(img_msg.header)

    ap = argparse.ArgumentParser()
    ap.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
    ap.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    args = vars(ap.parse_args())

    
    if ARUCO_DICT.get(args["type"], None) is None:
        #print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)

    aruco_dict_type = ARUCO_DICT[args["type"]]
    calibration_matrix_path = args["K_Matrix"]
    distortion_coefficients_path = args["D_Coeff"]
    
    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    try:
        cv_img = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    cv_img = cv2.flip(cv_img, -1)
    cv2.imshow("go1 eye", cv_img)
    cv2.waitKey(3)

    frame = cv_img

    output, rvec, tvec = pose_esitmation(frame, aruco_dict_type, k, d)
    print(rvec, tvec)

    cv2.imshow('Estimated Pose', output)

    #key = cv2.waitKey(1) & 0xFF
    #if key == ord('q'):
        #break

    #cv2.imwrite("go1"+str(img_msg.header.seq)+".jpg",cv_img)
    # count = count + 1


rospy.init_node('listener', anonymous=True)
sub_image = rospy.Subscriber("/camera/front", Image, image_callback)

cv2.namedWindow("go1 eye", 1)
while not rospy.is_shutdown():
    rospy.spin()
    

