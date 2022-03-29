#! /usr/bin/env python
#Ubuntu 16.04, ROS Kinetic, python 2.7
#Script to adapt RGB-D data in png files to ROS msgs in bag file: sensor_msgs/CameraInfo, sensor_msgs/Image, sensor_msgs/PointCloud2

###ROS CameraInfo message format###          ###ROS Image message format###          ###ROS PointCloud2 message format###
# std_msgs/Header header                                   # std_msgs/Header header                          # std_msgs/Header header
# uint32 height                                                         # uint32 height                                               # uint32 height
# uint32 width                                                          # uint32 width                                                 # uint32 width
# storing distortion_model                                     # string encoding                                           # sensor_msgs/PointField []
# float64 [] D                                                             # uint8 is_bigendian                                     # bool is_bigendian
# float64 [9] K                                                           # uint32 step                                                  # uint32 point_step
# float64 [9] R                                                          # uint8 [] data                                                  # uint32 row_step
# float64 [12] P                                                                                                                                  # uint8 [] data
# uint32 binning_x                                                                                                                            # bool is_dense
# uint32 binning_y
# sensor_msgs/RegionsOfInterest roi

import os
import rospy
import rosbag
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from tf2_msgs.msg import TFMessage 
from geometry_msgs.msg import TransformStamped
import tf
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
import rospkg
import std_msgs.msg

####CONFIGURABLE PARAMETERS####
scenario = "alma" #get this data from launch file

#Camera calibration parameters, taken from dataset
cx = 157.3245865
cy = 120.0802295
fx = 286.441384
fy = 271.36999


####FILE PRE-PROCESSING####
#1. Obtain images from folder
path_imgs= "/home/fer/Desktop/catkin_ws/src/ROS_AMCL_Hybrid_Localization/rgbd_data_adapter/data/alma/fullhose1_rgbd/"
file_rgbd_tstamps ="/home/fer/Desktop/catkin_ws/src/ROS_AMCL_Hybrid_Localization/rgbd_data_adapter/data/alma/fullhose1_rgbd.txt"

filenames = []
rgb_filenames = [ ]
depth_filenames = [ ]

#1.1 Divide rgb and depth images 
for file in os.listdir(path_imgs):
    if file.endswith( '.png'):
        filenames.append(file) #append in list all png files located in the given path
    if 'depth' in file:
        depth_filenames.append(file)
    if 'intensity' in file:
        rgb_filenames.append(file)
        
num_files = len(filenames)
print 'There are', num_files, 'depth and image png files in the folder in path: \n', path_imgs
filenames.sort(key=lambda f: int(filter(str.isdigit, f))) #order files names in natural ascending order
depth_filenames.sort(key=lambda f: int(filter(str.isdigit, f))) #order files names in natural ascending order
rgb_filenames.sort(key=lambda f: int(filter(str.isdigit, f))) #order files names in natural ascending order

#1.2 Logic to put timestamp data transformed from TTimeStamp format to unix epoch
with open(file_rgbd_tstamps,'r') as tstamps_file:
    tstamp_file= tstamps_file.readlines()
    #print "The num of lines in the tstamp file is", len(tstamp_file) #first four lines are not relevant just header data
    tstamp_lines = tstamp_file[4:len(tstamp_file)]
    tstamp = []
    for line in tstamp_lines:
        breaking_lines = line.split()
        tstamp.append(breaking_lines[8]) #tstamp data in 9th row
    for item in range(0,len(tstamp)):
        mrpt_tstamp = int(tstamp[item]) #MRPT TTimeStamp format must be converted to ROS compatible timestamps
        ros_secs = (mrpt_tstamp/10000000) - (11644473600) #formulas taken from: http://docs.ros.org/en/jade/api/mrpt_bridge/html/time_8h_source.html#l00027
        ros_nsecs =  (mrpt_tstamp % 10000000) * 100
        tstamp[item]=rospy.Time(ros_secs,ros_nsecs)#turning the timestamp values to timestamp object  
  
#1.3 Logic to put RGB-D imgs from the 4 different cameras in separate lists  
    rgbd_id_lines = tstamp_file[4:len(tstamp_file)]
    rgbd_id = []
    for line in rgbd_id_lines:
        breaking_lines = line.split()
        rgbd_id.append(breaking_lines[1]) #rgbd_id data in 2th row

RGB_id1_file_name =  [[], []]
RGB_id2_file_name =  [[], []]
RGB_id3_file_name =  [[], []]
RGB_id4_file_name =  [[], []]

D_id1_file_name =  [[], []]
D_id2_file_name =  [[], []]
D_id3_file_name =  [[], []]
D_id4_file_name =  [[], []]

for i in range(0,len(rgbd_id)):
    if '4' in rgbd_id[i]:
        RGB_id4_file_name[0].append(rgb_filenames[i])
        RGB_id4_file_name[1].append(tstamp[i])
        D_id4_file_name[0].append(depth_filenames[i])
        D_id4_file_name[1].append(tstamp[i])
    if '3' in rgbd_id[i]:
        RGB_id3_file_name[0].append(rgb_filenames[i])
        RGB_id3_file_name[1].append(tstamp[i])
        D_id3_file_name[0].append(depth_filenames[i])
        D_id3_file_name[1].append(tstamp[i])
    if '2' in rgbd_id[i]:
        RGB_id2_file_name[0].append(rgb_filenames[i])
        RGB_id2_file_name[1].append(tstamp[i])
        D_id2_file_name[0].append(depth_filenames[i])
        D_id2_file_name[1].append(tstamp[i])
    if '1' in rgbd_id[i]:
        RGB_id1_file_name[0].append(rgb_filenames[i])
        RGB_id1_file_name[1].append(tstamp[i])
        D_id1_file_name[0].append(depth_filenames[i])
        D_id1_file_name[1].append(tstamp[i])
        
print "number of timestamps registred   ", len(tstamp)
print "number of rgbd readings registred", len(rgbd_id)
print "number of readings with RGBD_id1", len(RGB_id1_file_name[0])
print "number of readings with RGBD_id2", len(RGB_id2_file_name[0])
print "number of readings with RGBD_id3", len(RGB_id3_file_name[0])
print "number of readings with RGBD_id4", len(RGB_id4_file_name[0])

####MOSAIC CREATION####
#Crear grupos de 4 imagenes
# path_imgs= "/home/fer/Desktop/catkin_ws/src/ROS_AMCL_Hybrid_Localization/rgbd_data_adapter/data/alma/"
# images = []
# for file in os.listdir(path_imgs):
    # if file.endswith( '.png'):
        # image = cv2.imread(path_imgs+file)
        # cv2.imshow("image",image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # images.append(image)
# print images
# stitcher = cv2.createStitcher()
# (status, stitched) = stitcher.stitch(images)

# if status == 0:
    # cv2.imwrite(args["output"], stitched)
    # cv2.imshow("Stitched", stitched)
    # cv2.waitKey(0)


#### FUNCTIONS TO FILL RELEVANT DATA ####

#Function to populate ROS Image msg format
def fill_image_msg(img_path, seq, t):  
    cv_img = cv2.imread(img_path)     #Use CVbridge to convert image in given path to ros img msg
    cv_img = cv2.rotate(cv_img, cv2.ROTATE_90_COUNTERCLOCKWISE) #rotate image to see it straight in rviz
    height, width = cv_img.shape[:2]
    img_msg_data = bridge.cv2_to_imgmsg(cv_img, encoding="bgr8") #verificar que este bien bgr8
    img_msg_data.header.seq = seq     #Fill additional ros image msg information
    img_msg_data.header.stamp = t
    img_msg_data.header.frame_id = '/camera/RGB/Image' #transform frame name
    img_msg_data.height = height
    img_msg_data.width = width
    img_msg_data.encoding = 'bgr8' #verificar que este bien bgr8    
    return img_msg_data

#Function to populate CameraInfo message
# def fill_CameraInfo_msg(img_path, seq, t):                      
    # cam_info = CameraInfo()
    # cam_info.header.seq = seq
    # cam_info.header.stamp = t
    # cam_info.header.frame_id =  '/camera/RGB/CameraInfo'
    # cam_info.height = 
    # cam_info.width =
    # cam_info.distortion_model = 
    # cam_info.D = 
    # cam_info.K = 
    # cam_info.R = 
    # cam_info.P = 
    # cam_info.binning_x =
    # cam_info.binning_y =
    # cam_info.RegionsOfInterest = 
        
#Function to populate point cloud message
# def fill_pointcloud_msg(img_path, cx, cy, fx, fy, seq, t):
    # cv_img = cv2.imread(img_path)     #Use CVbridge to convert image in given path to ros img msg
    # cv_img = cv2.rotate(cv_img, cv2.ROTATE_90_COUNTERCLOCKWISE) #rotate image to see it straight in rviz
    # height, width = cv_img.shape[:2]
    
    # pointcloud = PointCloud2()
    
    # header = std_msgs.msg.Header()
    # header.stamp = t
    # header.seq = seq
    # header.frame_id =  '/camera/RGB/CameraInfo'
    
    # # pointcloud.height = height
    # # pointcloud.width = width
    # # pointcloud.is_bigendian = False #CONFIRMAR assumption
    # # pointcloud.is_dense = True #True if there are no invalid points
    # # pointcloud.point_step = 24 #CONFIRMAR, length of a point in bytes
    # # pointcloud.row_step = pointcloud.point_step * width # length of a row in bytes
    # # # sensor_msgs/PointField []
    # # # uint8 [] data #list  of points encoded as byte stream, each point is a struct
    
    # # Extract x,y,z data from depth image. Equations taken from dataset webpage
    # point = [[],[],[]]
    # points_list = []
    # for v in range(0,height):
        # for u in range (0, width):
            # print "DEBUG", type(cv_img[u][v])
            # print "DEBUG", cv_img[u][v]
            # print "DEBUG u is", u
            # print "DEBUG v is", v
            # #print cv_img
            
            # depth = cv_img[u][v] * (1/6,553.5) # read each single pixel in image
            # x = depth
            # y = (cx- u) * x/fx 
            # z = (cy- v) * x/fy  #CONFIRMAR, como rote la img tal vez van al reves las formulas
            # point[0].append(x)
            # point[1].append(y)
            # point[2].append(z)
            # points_list.append(point)
            
    # pointcloud2msg = pointcloud.create_cloud_xyz32(header, points)
    
    # return pointcloud2msg
         


 #Function to create TF odometry data
def create_TFmsg(x, y, z, roll, pitch, yaw, frame, child_frame, t, seq):
    trans = TransformStamped()
    trans.header.seq = seq
    trans.header.stamp = t
    trans.header.frame_id = frame
    trans.child_frame_id = child_frame
    trans.transform.translation.x = x
    trans.transform.translation.y = y
    trans.transform.translation.z = z
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    trans.transform.rotation.x = q[0]
    trans.transform.rotation.y = q[1]
    trans.transform.rotation.z = q[2]
    trans.transform.rotation.w = q[3]
    msg = TFMessage()
    msg.transforms.append(trans)
    return msg

#### CREATE BAG FILE AND FILL ROS MSGS####

bag = rosbag.Bag('rgbd_data_'+scenario+'.bag', 'w') # Open bag file to write data in it 
bridge = CvBridge()


for i in range(0,len(RGB_id1_file_name[0])):

    img_path = path_imgs + RGB_id1_file_name[0][i]   
    dep_path = path_imgs + D_id1_file_name[0][i]
    tstamp_rgb1 = RGB_id1_file_name[1][i]   

    #Image message
    img_msg = fill_image_msg(img_path, i, tstamp_rgb1)
    
    #PointCloud message
    #pointcloud_msg = fill_pointcloud_msg(dep_path, cx, cy, fx, fy, i, tstamp_rgb1)
   
    #Calibration camera message 
    
    #TF data 
    tf_data = create_TFmsg(0.271, -0.031, 1.045, 90, 0, -45, '/base_link', '/camera/RGB/Image', tstamp_rgb1, i)
    #AGREGAR topic del pointcloud
     
    #Write data in bag
    bag.write('/camera/RGB/Image', img_msg, tstamp_rgb1)
    #bag.write('/camera/RGB/PointCloud', pointcloud_msg, tstamp_rgb1) #CONFIRMAR nombre del topic
    bag.write('/tf', tf_data, tstamp_rgb1)
    
bag.close()
