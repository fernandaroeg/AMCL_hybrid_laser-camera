#! /usr/bin/env python
#Ubuntu 16.04, ROS Kinetic, python 2.7
import rospy
import rosbag
from sensor_msgs.msg import LaserScan
import os
from math import pi
from tf2_msgs.msg import TFMessage 
from geometry_msgs.msg import TransformStamped
import tf

###ROS LaserScan message format###
# std_msgs/Header header
  # uint32 seq
  # time stamp
        #sec            
        #nsecs          
  # string frame_id
# float32 angle_min
# float32 angle_max
# float32 angle_increment
# float32 time_increment
# float32 scan_time
# float32 range_min
# float32 range_max
# float32[] ranges 
# float32[] intensities

#2. File Paths and Parameters
path_laser_logs   = rospy.get_param('/laser_data_adapter/laser_logs_path')
file_laser_tstamps= rospy.get_param('/laser_data_adapter/tstamp_file')
scenario          = rospy.get_param('/laser_data_adapter/environment')


#3. Get all txt files names in a list
filenames = [ ]
for file in os.listdir(path_laser_logs):
    if file.endswith( '.txt'): 
        filenames.append(file) #append in list all text files located in the given path_laser_logs
        
num_files = len(filenames)
print 'There are', num_files, 'laser txt files in the folder path_laser_logs'
filenames.sort(key=lambda f: int(filter(str.isdigit, f))) #order files names in natural ascending order

#4. Function to populate ROS LaserMsg format with relevant data
def fill_laser_msg(laser_values, aperture, num_txt_file, num_readings, t):  
    laser_msg = LaserScan()
    laser_msg.header.seq = num_txt_file
    laser_msg.header.stamp = t
    laser_msg.header.frame_id = '/laser' #transform frame name
    laser_msg.angle_min = aperture * -0.5 #formulas (lines 50-54) taken from https://github.com/mrpt-ros-pkg/mrpt_bridge/blob/ros1/src/laser_scan.cpp
    laser_msg.angle_max = aperture *  0.5  
    laser_msg.angle_increment = aperture/(num_readings-1)
    laser_msg.time_increment = 1/30 
    laser_msg.scan_time = 0.0 
    laser_msg.range_min = 0.06 # taken from URG-04LX-UG01 hokuyo laser datasheet https://hokuyo-usa.com/application/files/5115/8947/8197/URG-04LX-UG01_Specifications_Catalog.pdf
    laser_msg.range_max = 4.0  # taken from URG-04LX-UG01 hokuyo laser datasheet
    laser_msg.ranges = laser_values
    return laser_msg
    
 #5. Function to create TF data for laser frame
def laser_tf_msg(seq, t):
    trans = TransformStamped()
    trans.header.seq = seq
    trans.header.stamp = t
    trans.header.frame_id = '/base_link'
    trans.child_frame_id = '/laser'
    trans.transform.translation.x = 0.205 #laser position in m, from dataset paper http://mapir.uma.es/papersrepo/2017/2017-raul-IJRR-Robot_at_home_dataset.pdf
    trans.transform.translation.y = 0.0
    trans.transform.translation.z = 0.31
    #q = tf.transformations.quaternion_from_euler(yaw,pitch,roll)
    q = tf.transformations.quaternion_from_euler(0,0,0)
    trans.transform.rotation.x = q[0]
    trans.transform.rotation.y = q[1]
    trans.transform.rotation.z = q[2]
    trans.transform.rotation.w = q[3]
    msg = TFMessage()
    msg.transforms.append(trans)
    return msg
    
#6. Logic to put timestamp data in list format and transform from MRPT TTimeStamp format to unix epoch
with open(file_laser_tstamps,'r') as tstamps_file:
    tstamp_file= tstamps_file.readlines()
    #print "The num of lines in the tstamp file is", len(tstamp_file) #first four lines are not relevant just header data
    tstamp_lines = tstamp_file[4:len(tstamp_file)]
    tstamp = []
    for line in tstamp_lines:
        breaking_lines = line.split()
        tstamp.append(breaking_lines[8]) #tstamp data in 8th row
    for item in range(0,len(tstamp)):
        mrpt_tstamp = int(tstamp[item]) #MRPT TTimeStamp format must be converted to ROS compatible timestamps
        ros_secs = (mrpt_tstamp/10000000) - (11644473600) #formulas taken from: http://docs.ros.org/en/jade/api/mrpt_bridge/html/time_8h_source.html#l00027
        ros_nsecs =  (mrpt_tstamp % 10000000) * 100
        tstamp[item]=rospy.Time(ros_secs,ros_nsecs)#turning the timestamp values to timestamp object

#7.  Open bag file to write data in it 
bag = rosbag.Bag('laser_data_'+scenario+'.bag', 'w')

#8. Extract  laser data from multiple text files (one text file for each scan with multiple laser readings )
for file in range(num_files):
    with open (path_laser_logs+filenames[file], 'r') as myfile:
        data = myfile.readlines()
        laser_aperture = float(data[7])
        range_values = data[10]
        range_values_list= range_values.split()
        map_r_values = map(float, range_values_list)
        range_values = list(map_r_values)
        num_scans = len(range_values)
        print "There are ", num_scans, "scanner readings in txt file", file
        laser_msg = fill_laser_msg(range_values, laser_aperture, file,num_scans, tstamp[file])  #call function to fill laser data
        tf_data = laser_tf_msg(file,tstamp[file])#call function to generate TF laser data
    bag.write("/tf", tf_data, tstamp[file])
    bag.write("/scan", laser_msg, tstamp[file])


bag.close() #export rosbag file to /home/user/.ros 