import rospy
import numpy as np
import cv2
import sys
import argparse
import time
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32
# from unitree_legged_msgs.msg import HighCmd
import os

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def rmatrix2euler(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

class ControlNode(object):
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.pub = rospy.Publisher('chatter', Int32, queue_size=10)
        self.sub_image = rospy.Subscriber("/camera/front", Image, self.image_callback)
        self.bridge = CvBridge()
      

    def pose_esitmation(self, frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

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
        rotation_angle = None
        rotation_axis = None
        tvec = None
        yaw_angle = None
            # If markers are detected
        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                # large aruco length: 0.511
                # smaller one: 0.109
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.511, matrix_coefficients,
                                                                        distortion_coefficients)
                # Draw a square around the markers
                cv2.aruco.drawDetectedMarkers(frame, corners) 
                rotation_angle = np.linalg.norm(rvec)
                rotation_axis = rvec / rotation_angle
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                euler_angles = rmatrix2euler(rotation_matrix)
                print(tvec)
                yaw_angle = euler_angles[0] + math.pi
                yaw_angle = math.degrees(yaw_angle)

        return frame, yaw_angle, tvec

    def image_callback(self, img_msg):
        rospy.loginfo(img_msg.header)
        
        if ARUCO_DICT.get(args["type"], None) is None:
            print("ArUCo tag type is not supported")
            sys.exit(0)

        aruco_dict_type = ARUCO_DICT[args["type"]]
        calibration_matrix_path = args["K_Matrix"]
        distortion_coefficients_path = args["D_Coeff"]
        
        k = np.load(calibration_matrix_path)
        d = np.load(distortion_coefficients_path)

        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

     
        output, yaw_angle, tvec = self.pose_esitmation(cv_img, aruco_dict_type, k, d)
        if tvec is None:
            #cmd_type = 4 will be the robot not seeing the marker and turning slowly in place
            cmd_type = 4;
            print(cmd_type)
            self.pub.publish(cmd_type)
        else:
            tvec = tvec.squeeze()
            print("yaw =", yaw_angle,"tvec =", tvec)

            cv2.imshow('Estimated Pose', output)
            cv2.waitKey(3)
        
            dist = tvec[2]
            x_offset = tvec[0]

            if abs(x_offset) < 0.2 and dist  > 0.6:
                #cmd_type = 1 will be the robot walking forward       
                cmd_type = 1;
                print(cmd_type)
                self.pub.publish(cmd_type)
            elif x_offset < -0.2 and dist > 0.5:
                #robot turn clockwise and walk forward
                cmd_type = 2;
                print(cmd_type)
                self.pub.publish(cmd_type)
            elif x_offset > 0.2 and dist > 0.5:
                #robot turn countre clockwise and walk forward
                cmd_type = 3;
                print(cmd_type)
                self.pub.publish(cmd_type)
            # elif dist > 0.5 and tvec is None:
            #     #cmd_type = 4 will be the robot not seeing the marker and turning slowly in place
            #     cdm_type = 4;
            #     print(cmd_type)
            #     self.pub.publish(cmd_type)
            else:
                #cmd_type = 0 will be the robot not moving     
                cmd_type = 0;
                print(cmd_type)
                self.pub.publish(0)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
    ap.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    args = vars(ap.parse_args())


    rosnode = ControlNode()
    while not rospy.is_shutdown():
        rospy.spin()
    

