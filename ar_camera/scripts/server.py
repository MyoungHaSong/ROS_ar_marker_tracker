#!/usr/bin/env python

from obj_msg.srv import CameraRequests,CameraRequestsResponse 
import rospy
import numpy as np
import cv2, PIL
from cv2 import aruco
import math
import sys
import os
#####################
### ROS
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import PointCloud2 as pc2_msg
import sensor_msgs.point_cloud2 as pc2
import pcl_ros as pcl
from geometry_msgs.msg import Pose, PoseArray

fdir = os.path.dirname(os.path.realpath(__file__))
mat_path = os.path.join(fdir, 'matrix.npy')
dis_path = os.path.join(fdir, 'distortion.npy')

#cameramatrix = np.load(mat_path)
#dist_coeffs = np.load(dis_path)
cameramatrix =  np.array([[ 617.536865234375,    0.0, 317.089599609375],
                          [    0., 616.9095458984375, 245.19430541992188],
                          [    0.,    0.,           1.]])
dist_coeffs = np.zeros((5,1))
def eulerAnglesToRotationMatrix(theta) :
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])



    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])

    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])


    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R


l_1=1305.6/1000
l_2=77.19/1000
l_3=35/1000
l_4=(20.5-15)/1000
l_5=(20.05-1.1)/1000
th = 30
sind = math.sin(math.radians(30))
cosd = math.cos(math.radians(30))

bridge = CvBridge()


def transform_coor(x_c,y_c,z_c):
    t_x = l_2+l_4*sind+l_5*cosd-y_c*sind+z_c*cosd
    t_y = -x_c
    t_z = l_1+l_3+l_4*cosd-l_5*sind-y_c*cosd-z_c*sind
    return t_x, t_y, t_z


def transform_coor(x_c, y_c, z_c):
    input_pc = np.array([[x_c], [y_c], [z_c], [1]])
    trans_mat = np.array([[-0.00043854, -0.46793108, 0.88376485, 0.090225],\
                          [-0.99904221, 0.03887386, 0.02008698, 0.021119],\
                          [-0.04375467, -0.88290958, -0.46749995, 1.240883]])
    output_pc = np.dot(trans_mat, input_pc)

    return output_pc[0], output_pc[1], output_pc[2]


def ar_marker_pose_estimation(req):
    print("Start AR Marker Pose Estimation")
    ar_pub = rospy.Publisher('/AR/Estimation_image', Image, queue_size = 10)
    pose_pub = rospy.Publisher('/AR/Estimation_pose', PoseArray, queue_size = 10)
    try:
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict, parameters=parameters)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        for corner in corners:
            cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)
        frame_markers = aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)
        markerLength = 0.07
        flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, cameramatrix, dist_coeffs)
        total_rvec = np.zeros([60,1,3])
        total_tvec = np.zeros([60,1,3])
        
        length_of_axis = 0.05
        total_center = np.zeros(shape = [60,1,2])

        for index in range(len(ids)):
            center = np.array(np.mean(corners[index][0],axis = 0), dtype = np.int)
            total_center[ids[index]-1] = center
            total_rvec[ids[index]-1] = rvecs[index]
            total_tvec[ids[index]-1] = tvecs[index]
        imaxis = aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)
        for i in range(len(tvecs)):
            imaxis = aruco.drawAxis(imaxis, cameramatrix, dist_coeffs, rvecs[i], tvecs[i], length_of_axis)
        # Display the resulting frame
        ar_pub.publish(bridge.cv2_to_imgmsg(imaxis,'bgr8'))
        ## get only 1 point data from point clouds
        ## uvs must be iterable, so use 2d list. 
        pose_arr = PoseArray()
        index_ids = ids.reshape(ids.shape[0])
        for point_i in index_ids:
            pixel_x = int(total_center[point_i-1][0][0])
            pixel_y = int(total_center[point_i-1][0][1])
            for p in pc2.read_points(ros_cloud,  field_names = ("x", "y", "z"), uvs=[[pixel_x,pixel_y]]):
                # print('{}  p.x: {}'.format(ids[point_i],p[0]))
                # print('{}  p.y: {}'.format(ids[point_i],p[1]))
                # print('{}  p.z: {}'.format(ids[point_i],p[2]))
                
                pose_msg = Pose()
                qx = total_rvec[point_i-1][0][0]
                qy = total_rvec[point_i-1][0][1]
                qz = total_rvec[point_i-1][0][2]
                if math.isnan(p[0]) : 
                    print('is nan')
                    p = [0,0,0]
                    p[0] = total_tvec[point_i-1][0][0]
                    p[1] = total_tvec[point_i-1][0][1]
                    p[2] = total_tvec[point_i-1][0][2]
                tx, ty,tz = transform_coor(p[0],p[1],p[2])
                pose_msg.position.x = tx
                pose_msg.position.y = ty
                pose_msg.position.z = tz
                pose_msg.orientation.x = qx
                pose_msg.orientation.y = qy
                pose_msg.orientation.z = qz
    
                pose_msg.orientation.w = point_i
                pose_arr.poses.append(pose_msg)
        pose_pub.publish(pose_arr)

        # When everything done, release the capture
    except TypeError as e :
        print(e)
    return CameraRequestsResponse(pose_arr)


def table_callback(rgb_img,point_cloud):
    cv_image = bridge.imgmsg_to_cv2(rgb_img,'bgr8')
    ros_cloud = point_cloud
    global cv_image, ros_cloud


def AR_Marker_Pose_Estimation():
    rospy.init_node('AR_Marker')
    s = rospy.Service('/AR_Marker_Service',CameraRequests, ar_marker_pose_estimation)
    rgb_sub = message_filters.Subscriber('/camera/color/image_raw',Image)
    depth_sub = message_filters.Subscriber('/camera/depth_registered/points',pc2_msg)
    ts = message_filters.TimeSynchronizer([rgb_sub, depth_sub], queue_size = 10)
    ts.registerCallback(table_callback)
    rospy.spin()



if __name__ == "__main__":
    AR_Marker_Pose_Estimation()
