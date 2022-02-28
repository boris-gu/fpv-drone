#!/usr/bin/env python3

# =====================
#  Зависнуть у маркера
# =====================

import numpy as np
import cv2
import cv2.aruco as aruco
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from aruco_calibration import Calibration as clb
from drone_api import Drone_api

FONT = cv2.FONT_HERSHEY_PLAIN


def callback(data):
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    except CvBridgeError as e:
        rospy.loginfo(e)
        return

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #_, gray = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict,
                                                          parameters=parameters,
                                                          cameraMatrix=camera_matrix,
                                                          distCoeff=dist_coef)
    global marker_pose
    if np.all(ids is not None):
        # TODO: ПОЧИНИТЬ (возможно только калибровка)
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners, 0.2, camera_matrix,
                                                                   dist_coef)
        x, y, z, _ = drone.get_pose()
        marker_x = tvec[0][0][2] + x
        marker_y = -tvec[0][0][0] + y
        marker_z = -tvec[0][0][1] + z
        marker_pose = [marker_x, marker_y, marker_z]
        for i in range(0, len(ids)):
            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix,
                           dist_coef, rvec[i], tvec[i], 0.2)
        cv2.putText(frame, ' id' + str(ids[i])[1:-1], (20, 30), FONT,
                    1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, ' id' + str(ids[i])[1:-1], (20, 30), FONT,
                    1, (0, 0, 0), 1, cv2.LINE_AA)
        distance = np.sum(tvec ** 2, axis=2) ** 0.5
        for i in range(len(distance)):
            cv2.putText(frame, str([x, y, z]),
                        (20, 70+20*i), FONT, 1, (255, 255, 255),
                        3, cv2.LINE_AA)
            cv2.putText(frame, str([x, y, z]),
                        (20, 70+20*i), FONT, 1, (0, 0, 0),
                        1, cv2.LINE_AA)
    else:
        cv2.putText(frame, 'NOT FOUND', (20, 30), FONT,
                    1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, 'NOT FOUND', (20, 30), FONT,
                    1, (0, 0, 0), 1, cv2.LINE_AA)
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
        #image_pub.publish(bridge.cv2_to_imgmsg(gray, '8UC1'))
    except CvBridgeError as e:
        rospy.loginfo(e)


marker_pose = [0, 0, 2]

# calibration_save.yaml - уже проведена калибровка
camera_matrix, dist_coef = clb.loadCoefficients('calibration_save.yaml')
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()

drone = Drone_api()
drone.start()
print('OK1')
image_sub = rospy.Subscriber('/iris_front_fpv/usb_cam/image_raw',
                             Image, callback, queue_size=1)
rospy.loginfo('Start Subscriber')
image_pub = rospy.Publisher('/iris_front_fpv/usb_cam/location_img',
                            Image, queue_size=1)
rospy.loginfo('Start Publisher')

drone.sleep(5)
print('OK2')
drone.set_local_pose(0, 0, 2)
print('OK3')
while not drone.point_is_reached() and not drone.is_shutdown():
    drone.sleep(0.5)
print('OK5')
while not drone.is_shutdown():
    drone_point = marker_pose
    drone.set_local_pose(drone_point[0]-1, drone_point[1], drone_point[2])
    #drone.set_local_pose(0, 0, 2)
    drone.sleep(0.5)
drone.stop()
