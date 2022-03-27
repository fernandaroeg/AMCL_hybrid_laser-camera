#! /usr/bin/env python
#Ubuntu 16.04, ROS Kinetic, python 2.7
#Script to adapt RGB-D data in png files to ROS msgs in bag file: sensor_msgs/CameraInfo, sensor_msgs/Image, sensor_msgs/PointCloud2
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

####FILE PRE-PROCESSING####
#1. Obtain images from folder
path_imgs= "/home/fer/Desktop/catkin_ws/src/ROS_AMCL_Hybrid_Localization/rgbd_data_adapter/data/alma/fullhose1_rgbd/"
file_rgbd_tstamps ="/home/fer/Desktop/catkin_ws/src/ROS_AMCL_Hybrid_Localization/rgbd_data_adapter/data/alma/fullhose1_rgbd.txt"

filenames = []
rgb_filenames = [ ]
depth_filenames = [ ]

for file in os.listdir(path_imgs):
    if file.endswith( '.png'):
        filenames.append(file) #append in list all png files located in the given path
    if 'depth' in file:
        depth_filenames.append(file)
    if 'intensity' in file:
        rgb_filenames.append(file)
        
num_files = len(filenames)
print 'There are', num_files, 'png files in the folder path', path_imgs
filenames.sort(key=lambda f: int(filter(str.isdigit, f))) #order files names in natural ascending order
depth_filenames.sort(key=lambda f: int(filter(str.isdigit, f))) #order files names in natural ascending order
rgb_filenames.sort(key=lambda f: int(filter(str.isdigit, f))) #order files names in natural ascending order

#1.2 Logic to put RGB-D ids in list format and timestamp data transformed from TTimeStamp format to unix epoch
with open(file_rgbd_tstamps,'r') as tstamps_file:
    tstamp_file= tstamps_file.readlines()
    #print "The num of lines in the tstamp file is", len(tstamp_file) #first four lines are not relevant just header data
    
    rgbd_id_lines = tstamp_file[4:len(tstamp_file)]
    rgbd_id = []
    for line in rgbd_id_lines:
        breaking_lines = line.split()
        rgbd_id.append(breaking_lines[1]) #rgbd_id data in 2th row
    
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
        
print "timestamps number", len(tstamp)
print "rgbd id number", len(rgbd_id)
id1_list = []
id2_list = []
id3_list = []
id4_list = []
for item in rgbd_id:
    if '4' in item:
        id4_list.append(item)
    if '3' in item:
        id3_list.append(item)
    if '2' in item:
        id2_list.append(item)
    if '1' in item:
        id1_list.append(item)
print "id 1 number of files", len(id1_list)
print "id 2 number of files", len(id2_list)
print "id 3 number of files", len(id3_list)
print "id 4 number of files", len(id4_list)

#Crear grupos de 4 imagenes

####MOSAIC STITCHING####
path_prueba= "/home/fer/Desktop/catkin_ws/src/ROS_AMCL_Hybrid_Localization/rgbd_data_adapter/data/alma/"
images = []
for file in os.listdir(path_prueba):
    if file.endswith( '.png'):
        image = cv2.imread(path_prueba+file)
        cv2.imshow("image",image)
        time.sleep(1)
        images.append(image)
print images
stitcher = cv2.createStitcher()
(status, stitched) = stitcher.stitch(images)

if status == 0:
    cv2.imwrite(args["output"], stitched)
    cv2.imshow("Stitched", stitched)
    cv2.waitKey(0)
        
####POINTCLOUD CREATION####
# for image in folder	
	# read each single pixel in image
# for pixel in im_size 320x240
	# i=pixel
	# depth_i = value_of_pixel * (1/6, n)
	# x = depth_i
	# y = (cx-c) * x/fx
	# z = (cy-r) * x/fy
# start rosbag obj pointcloud
# rosbag.write(x,y,z, timestam, sequence)
# rosbag.close

####FILLING ROS MSGS####


#publish static camera frame

# # Function to populate ROS Image msg format with relevant data
# def fill_image_msg(img_data, t):  
    # img_msg = Image()
    # img_msg.header.seq = num_txt_file
    # img_msg.header.stamp = t
    # img_msg.header.frame_id = '/camera/RGB/Image' #transform frame name
    # img_msg.height = 
    # img_msg.width = 
    # img_msg.encoding = 
    # img_msg.data = 
    # return img_msg
        
# # Open bag file to write data in it 
# scenario = "alma" #!!!!! get this data from launch file
# bag = rosbag.Bag('laser_data_'+scenario+'.bag', 'w')

# #9. Extract  laser data from multiple text files (one text file for each scan with multiple laser readings )
# for file in range(num_files):
    # with open (path_laser_logs+filenames[file], 'r') as myfile:
        # data = myfile.readlines()
        # range_values = data[10]
        # range_values_list= range_values.split()
        # map_r_values = map(float, range_values_list)
        # range_values = list(map_r_values)
        # num_scans = len(range_values)
        # print "There are ", num_scans, "scanner readings in txt file", file
        # img_msg = fill_img_msg(range_values,file,num_scans, tstamp[file])  #call function to fill laser data
        # #odom_tf_data = make_tf_msg(x[file],y[file],theta[file],tstamp[file])#call function to generate TF odom data
    # bag.write("/scan", img_msg, tstamp[file])

# bag.close() #export rosbag file to /home/user/.ros 