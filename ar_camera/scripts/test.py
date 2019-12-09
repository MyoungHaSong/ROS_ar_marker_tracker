import cv2, PIL
import rospy
import numpy as np
from cv2 import aruco
import pandas as pd
import yaml
import math
import sys
### ROS
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import PointCloud2 as pc2_msg
import sensor_msgs.point_cloud2 as pc2
import pcl_ros as pcl
from geometry_msgs.msg import Pose, PoseArray



class table_eye_cali:
    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_sub = message_filters.Subscriber('/camera/color/image_raw',Image)
        self.depth_sub = message_filters.Subscriber('/camera/depth_registered/points',pc2_msg)
        self.ar_pub = rospy.Publisher('/AR/Estimation_image', Image, queue_size = 10)
        self.pose_pub = rospy.Publisher('/AR/Estimation_pose', PoseArray, queue_size = 10)
        self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size = 10)
        self.ts.registerCallback(self.table_callback)

    

    def table_callback(self, rgb_img,ros_cloud):
        try:
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_img,'bgr8')
            gray = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_250)
            parameters =  aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict, parameters=parameters)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
            for corner in corners:
                cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)
            frame_markers = aruco.drawDetectedMarkers(rgb_img.copy(), corners, ids)
            markerLength = 0.8
            flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, cameramatrix, dist_coeffs)
            # result_matrix = np.zeros(shape=[4,4])
            # rotation_matrix = eulerAnglesToRotationMatrix(rvecs[0][0])
            # result_matrix[:3,:3] = rotation_matrix
            # result_matrix[:3,3] = tvecs[0]
            # result_matrix[3,3] = 1.
            length_of_axis = 0.6
            total_center = np.zeros(shape = [4,1,2])
            for index in range(len(ids)):
                center = np.array(np.mean(corners[index][0],axis = 0), dtype = np.int)
                total_center[ids[index]-1] = center
            imaxis = aruco.drawDetectedMarkers(rgb_img.copy(), corners, ids)
            for i in range(len(tvecs)):
                imaxis = aruco.drawAxis(imaxis, cameramatrix, dist_coeffs, rvecs[i], tvecs[i], length_of_axis)
            # Display the resulting frame
            self.ar_pub.publish(self.bridge.cv2_to_imgmsg(imaxis,'bgr8'))

            print('height: {}'.format(ros_cloud.height))
            print('width: {}'.format(ros_cloud.width))

class table_eye_cali:
    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_sub = message_filters.Subscriber('/camera/color/image_raw',Image)
        self.depth_sub = message_filters.Subscriber('/camera/depth_registered/points',pc2_msg)
        self.ar_pub = rospy.Publisher('/AR/Estimation_image', Image, queue_size = 10)
        self.pose_pub = rospy.Publisher('/AR/Estimation_pose', PoseArray, queue_size = 10)
        self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size = 10)
        self.ts.registerCallback(self.table_callback)

    

    def table_callback(self, rgb_img,ros_cloud):
        try:
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_img,'bgr8')
            gray = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_250)
            parameters =  aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict, parameters=parameters)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
            for corner in corners:
                cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)
            frame_markers = aruco.drawDetectedMarkers(rgb_img.copy(), corners, ids)
            markerLength = 0.8
            flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, cameramatrix, dist_coeffs)
            # result_matrix = np.zeros(shape=[4,4])
            # rotation_matrix = eulerAnglesToRotationMatrix(rvecs[0][0])
            # result_matrix[:3,:3] = rotation_matrix
            # result_matrix[:3,3] = tvecs[0]
            # result_matrix[3,3] = 1.
            length_of_axis = 0.6
            total_center = np.zeros(shape = [4,1,2])
            for index in range(len(ids)):
                center = np.array(np.mean(corners[index][0],axis = 0), dtype = np.int)
                total_center[ids[index]-1] = center
            imaxis = aruco.drawDetectedMarkers(rgb_img.copy(), corners, ids)
            for i in range(len(tvecs)):
                imaxis = aruco.drawAxis(imaxis, cameramatrix, dist_coeffs, rvecs[i], tvecs[i], length_of_axis)
            # Display the resulting frame
            self.ar_pub.publish(self.bridge.cv2_to_imgmsg(imaxis,'bgr8'))

            print('height: {}'.format(ros_cloud.height))
            print('width: {}'.format(ros_cloud.width))


            ## get only 1 point data from point clouds
            ## uvs must be iterable, so use 2d list. 
            pose_array = PoseArray()
            
            for point_i in range(len(ids)):
                
                pixel_x = int(total_center[point_i][0][0])
                pixel_y = int(total_center[point_i][0][1])
                for p in pc2.read_points(ros_cloud,  field_names = ("x", "y", "z"), uvs=[[pixel_x,pixel_y]]):
                    # print('{}  p.x: {}'.format(ids[point_i],p[0]))
                    # print('{}  p.y: {}'.format(ids[point_i],p[1]))
                    # print('{}  p.z: {}'.format(ids[point_i],p[2]))
                    pose_msg = Pose()

                    tx, ty,tz = transform_coor(p[0],p[1],p[2])
                    pose_msg.position.x = tx
                    pose_msg.position.y = ty
                    pose_msg.position.z = tz
                    pose_msg.orientation.x = 0
                    pose_msg.orientation.y = 0
                    pose_msg.orientation.z = 0
                    pose_msg.orientation.w = ids[point_i]
                    pose_array.poses.append(pose_msg)
            self.pose_pub.publish(pose_array)
         # When everything done, release the capture
        except TypeError as e :
            print(e)
            ## get only 1 point data from point clouds
            ## uvs must be iterable, so use 2d list. 
            pose_array = PoseArray()
            
            for point_i in range(len(ids)):
                
                pixel_x = int(total_center[point_i][0][0])
                pixel_y = int(total_center[point_i][0][1])
                for p in pc2.read_points(ros_cloud,  field_names = ("x", "y", "z"), uvs=[[pixel_x,pixel_y]]):
                    # print('{}  p.x: {}'.format(ids[point_i],p[0]))
                    # print('{}  p.y: {}'.format(ids[point_i],p[1]))
                    # print('{}  p.z: {}'.format(ids[point_i],p[2]))
                    pose_msg = Pose()

                    tx, ty,tz = transform_coor(p[0],p[1],p[2])
                    pose_msg.position.x = tx
                    pose_msg.position.y = ty
                    pose_msg.position.z = tz
                    pose_msg.orientation.x = 0
                    pose_msg.orientation.y = 0
                    pose_msg.orientation.z = 0
                    pose_msg.orientation.w = ids[point_i]
                    pose_array.poses.append(pose_msg)
            self.pose_pub.publish(pose_array)
         # When everything done, release the capture
        except TypeError as e :
            print(e)