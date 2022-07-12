#! /usr/bin/env python
#Ubuntu 16.04, ROS Kinetic, python 2.7
#Script to adapt RGB-D data in png files to ROS msgs in bag file: sensor_msgs/CameraInfo, sensor_msgs/Image, sensor_msgs/PointCloud2

###ROS CameraInfo message format###          ###ROS Image message format###          ###ROS PointCloud2 message format###
# std_msgs/Header header                     # std_msgs/Header header                # std_msgs/Header header
# uint32 height                              # uint32 height                         # uint32 height
# uint32 width                               # uint32 width                          # uint32 width
# storing distortion_model                   # string encoding                       # sensor_msgs/PointField []
# float64 [] D                               # uint8 is_bigendian                    # bool is_bigendian
# float64 [9] K                              # uint32 step                           # uint32 point_step
# float64 [9] R                              # uint8 [] data                         # uint32 row_step
# float64 [12] P                                                                     # uint8 [] data
# uint32 binning_x                                                                   # bool is_dense
# uint32 binning_y
# sensor_msgs/RegionsOfInterest roi

import os
import rospy
import rosbag
import numpy as np
import time
import cv2
from cv_bridge import CvBridge
import std_msgs.msg
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs import point_cloud2
from tf2_msgs.msg import TFMessage 
from geometry_msgs.msg import TransformStamped
import tf
import struct
import ctypes
from datetime import datetime

####CONFIGURABLE PARAMETERS####
scenario = "alma" #get this data from launch file

#Camera calibration parameters, taken from dataset
cx = 157.3245865
cy = 120.0802295
fx = 286.441384
fy = 271.36999


####FILE PRE-PROCESSING####
#1. Obtain images from folder
path_imgs         ="/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/rgbd_data_adapter/data/alma/fullhose1_rgbd/"
file_rgbd_tstamps ="/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/rgbd_data_adapter/data/alma/fullhose1_rgbd.txt"

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
    bridge = CvBridge()
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
def fill_CameraInfo_msg(img_path, cx, cy, fx, fy, seq, t): 
    cv_img = cv2.imread(img_path)     #Use CVbridge to convert image in given path to ros img msg
    cv_img = cv2.rotate(cv_img, cv2.ROTATE_90_COUNTERCLOCKWISE) #rotate image to see it straight in rviz
    height, width = cv_img.shape[:2]  
    cam_info = CameraInfo()
    cam_info.header.seq = seq
    cam_info.header.stamp = t
    cam_info.header.frame_id = '/camera/RGB/CameraInfo'
    cam_info.height = height
    cam_info.width = width
    cam_info.distortion_model = "plumb_bob"; #most common model used
    cam_info.D = [0, 0, 0, 0, 0] #Distortion parameters, for plumb_bob [k1,k2,t1, t2, k3] PENDING confirmar valores
    cam_info.K = np.array([fx,  0, cx,
                            0, fy, cy,
                            0,  0,  1])
    cam_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1] #PENDING confirmar, this seems to be the standard for stereo cams
    cam_info.P = np.array([fx,  0, cx, 0,
                            0, fy, cy, 0,
                            0,  0,  1, 0])
    cam_info.binning_x = 0 #default values=0 means no subsampling
    cam_info.binning_y = 0 
    #cam_info.RegionsOfInterest #Default value, width=height=0, means full resolution
    return cam_info

        
#Function to populate point cloud message
def fill_pointcloud_msg(img_path, cx, cy, fx, fy, seq, t, frame):
    #Image pre-processing
    cv_img = cv2.imread(img_path)     #Use CVbridge to convert image in given path to ros img msg
    cv_img = cv2.rotate(cv_img, cv2.ROTATE_90_COUNTERCLOCKWISE) #rotate image to see it straight in rviz
    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY) #reduce rgb dimension to grayscale
    height, width = cv_img.shape[:2]
    
    # Extract x,y,z data from depth image. Equations taken from dataset webpage    

    points= []
    for v in range(0,height):
        for u in range (0, width):
            pixel = cv_img[v][u]
            depth = pixel * (1/553.5) # read each single pixel in image
            x = depth
            z = (cx- v) * x/fx 
            y = (cy- u) * x/fy 
            point = [x*10, y*10, z*10]
            points.append(point)
    
    #print "points cloud len is", len(points)
    #print "points type is", type(points)
    #print "points 1st elem", points[0][0]
    #print "points is", points
    
    #Convert data to binary blob
    fields = [  PointField( 'x', 0, PointField.FLOAT32, 1),
                PointField( 'y', 4, PointField.FLOAT32, 1),
                PointField( 'z', 8, PointField.FLOAT32, 1)]
            
    cloud_struct = struct.Struct('<fff') #creating a struct instance to store data in bytes
                                                             #storing order: '<' little endian
                                                             #content to be stored: x (float), y (float), z (float) - fff
                                                        
    buff = ctypes.create_string_buffer(cloud_struct.size * len(points))
    point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
    offset = 0
    for p in points:
        pack_into(buff, offset, *p) 
        offset += point_step
   
   #Fill pointcloud message
    pointcloud = PointCloud2()
    header = std_msgs.msg.Header()
    header.stamp = t
    header.seq = seq
    header.frame_id =  frame 
    pointcloud.header = header
    pointcloud.height = 1
    pointcloud.width = len(points)
    pointcloud.is_dense = False #True if there are no invalid points
    pointcloud.is_bigendian = False
    pointcloud.fields = fields
    pointcloud.point_step = cloud_struct.size
    pointcloud.row_step = cloud_struct.size * len(points)
    pointcloud.data = buff.raw
        
    return pointcloud
         
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
for j in range (1,5):
    cam_num = j
    bag = rosbag.Bag('rgbd'+str(cam_num)+'_pc_'+scenario+'.bag', 'w') # Open bag file to write data in it
    #file = open('rgbd'+str(cam_num)+'_pc_'+scenario+'.txt', 'w') 
    
    if j == 1:
        rgb_file = RGB_id1_file_name
        d_file   = D_id1_file_name
        frame = '/pc1'
    elif j == 2:
        rgb_file = RGB_id2_file_name
        d_file   = D_id2_file_name
        frame = '/pc2'        
    elif j == 3:
        rgb_file = RGB_id3_file_name
        d_file   = D_id3_file_name 
        frame = '/pc3'
    elif j == 4:
        rgb_file = RGB_id4_file_name
        d_file   = D_id4_file_name
        frame = '/pc4'
    
    #for i in range(0,len(rgb_file[0])):
    for i in range(0,500):
    
        img_path = path_imgs + rgb_file[0][i]   
        dep_path = path_imgs + d_file[0][i]
        tstamp_rgb = rgb_file[1][i]      
        
        #Calibration camera message 
        ###cam_info_msg = fill_CameraInfo_msg(img_path, cx, cy, fx, fy, i, tstamp_rgb)
    
        #Image message
        ###img_msg = fill_image_msg(img_path, i, tstamp_rgb)
        
        #PointCloud message
        pointcloud_msg = fill_pointcloud_msg(dep_path, cx, cy, fx, fy, i, tstamp_rgb, frame)

    
        #TF data 
        if   j == 1:
            tf_data_pc = create_TFmsg(0.285,    0.0, 1.045, 0, 0, 0, '/base_link', frame, tstamp_rgb, i) #IMPORTANT el valor de roll lo pase de 90-0 ya que roto las imgs previamente con cv_bridge
            #Write data in bag                              
            bag.write(frame, pointcloud_msg, tstamp_rgb)    
            bag.write('/tf', tf_data_pc, tstamp_rgb)        
        elif j == 2:                                                               
            tf_data_pc = create_TFmsg(0.271, -0.031, 1.045, 0, 0, -45, '/base_link', frame, tstamp_rgb, i) 
            #Write data in bag                              
            bag.write(frame, pointcloud_msg, tstamp_rgb)    
            bag.write('/tf', tf_data_pc, tstamp_rgb)        
        elif j == 3:                                                               
            tf_data_pc = create_TFmsg(0.271,  0.031, 1.045, 0, 0, 45, '/base_link', frame, tstamp_rgb, i)
            #Write data in bag                              
            bag.write(frame, pointcloud_msg, tstamp_rgb)    
            bag.write('/tf', tf_data_pc, tstamp_rgb)         
        elif j == 4:                                                               
            tf_data_pc = create_TFmsg(0.240, -0.045, 1.045, 0, 0, -90, '/base_link', frame, tstamp_rgb, i)   
            #Write data in bag
            bag.write(frame, pointcloud_msg, tstamp_rgb)      
            bag.write('/tf', tf_data_pc, tstamp_rgb)
            
        #Write data in bag
        ###bag.write('/camera/RGB/Image',        img_msg, tstamp_rgb)
        ###bag.write('/camera/RGB/CamInfo', cam_info_msg, tstamp_rgb)
        ###bag.write('/tf',                   tf_data_im, tstamp_rgb)
        
        bag.write(frame, pointcloud_msg, tstamp_rgb)      
        bag.write('/tf', tf_data_pc, tstamp_rgb)
                
        #Export generated data for debugging purposes
        #ts = tstamp_rgb.to_time()
        #ts = int(ts)
        #ts = datetime.utcfromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S') # if "year is out of range" error, try `ts /= 1000`
        #file.write(ts)
        #file.write('\n')
        
    bag.close()
    #file.close() #close debugging txt file

