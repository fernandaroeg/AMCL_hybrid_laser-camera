#! /usr/bin/env python
#Ubuntu 16.04, ROS Kinetic, python 2.7
#Publish TF base_footprint data
import rospy
import rosbag
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage 
import tf
import numpy as np

#### 0. Parameter Setup ####
scenario = "alma"
bag_name = "mapTF"
        
#### 1. Logic to put timestamp data in list transformed from TTimeStamp format to unix epoch ####
file_tstamps ="/home/fer/Desktop/catkin_ws/src/ROS_AMCL_Hybrid_Localization/odometry_data_adapter_node/data/1_hokuyo_processed.txt"
with open(file_tstamps,'r') as tstamps_file:
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
    
 #### 2. Function to create TF data
def create_TFmsg(x, y, z, theta, frame, child_frame, t, seq):
    trans = TransformStamped()
    trans.header.seq = seq
    trans.header.stamp = t
    trans.header.frame_id = frame
    trans.child_frame_id = child_frame
    trans.transform.translation.x = x
    trans.transform.translation.y = y
    trans.transform.translation.z = z
    q = tf.transformations.quaternion_from_euler(0,0,theta)
    trans.transform.rotation.x = q[0]
    trans.transform.rotation.y = q[1]
    trans.transform.rotation.z = q[2]
    trans.transform.rotation.w = q[3]
    msg = TFMessage()
    msg.transforms.append(trans)
    return msg
    
##### 4. Open bag file to write in it ####
bag = rosbag.Bag(bag_name+'_'+scenario+'.bag', 'w')

for i in range(0,len(tstamp)):
    odom_tf_data = create_TFmsg(0.000602, 0.002559, 0.0, -0.002202, "/world", "/map",tstamp[i], i)
 
    #Write data to bag
    bag.write("/tf", odom_tf_data, tstamp[i])
    
bag.close() #export rosbag file to /home/user/.ros 
 
