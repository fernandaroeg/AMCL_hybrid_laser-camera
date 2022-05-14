#! /usr/bin/env python
#Ubuntu 16.04, ROS Kinetic, python 2.7
import rospy
import rosbag
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion,  TransformStamped
from tf2_msgs.msg import TFMessage 
import tf 
import numpy as np 

#SCRIPT TO CREATE BAG FILE WITH GROUND TRUTH DATA FROM DATASET LOG_ESTIMATED_PATH.TXT FILES 

#### 0. Parameter Setup ####
scenario    = rospy.get_param('/groundtruth_data_adapter/environment')
gtruth_file = rospy.get_param('/groundtruth_data_adapter/gtruth_file')
file_tstamps= rospy.get_param('/groundtruth_data_adapter/tstamp_file')


#### 1 Read data from ground truth with 2D pose #### #Archivo del data set que expresa x (metros), y (metros), theta (radianes)
with open(gtruth_file,'r') as gtruth_file:
    gtruth_file= gtruth_file.readlines()
    #print "The num of lines in the file is", len(gtruth_file)
    poseX = []
    poseY = []
    poseTheta = []
    for line in gtruth_file:
        breaking_lines = line.split()
        poseX.append(float(breaking_lines[1]))
        poseY.append(float(breaking_lines[2]))
        poseTheta.append(float(breaking_lines[3]))
        
#### 2 Logic to put timestamp data in list transformed from TTimeStamp format to unix epoch ####
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
    
 #### 3. Functions to create pose, marker and tf data
def create_pose_stamped(x, y, theta, t, seq):
    pose = PoseStamped()
    pose.header.seq = seq
    pose.header.stamp = t
    pose.header.frame_id = "/g_truth/Pose"
    quaternion_angle = tf.transformations.quaternion_from_euler(0, 0, theta)
    pose.pose = Pose( Point(x, y, 0.0),  Quaternion(*quaternion_angle) )  
    return pose

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
    
##### 4. Open bag file to write data in it ####
#List names with data to write from groundtruth: poseX, poseY, poseTheta, tstamp
bag = rosbag.Bag('groundtruth'+'_'+scenario+'.bag', 'w')
file = open('odom_debug.txt', 'w') #create txt file to export odom data for debugging purposes

for i in range(0,len(poseX)):
    path= Path()
    path.header.seq = i
    path.header.stamp = tstamp[i]
    path.header.frame_id = "/map"
    pose_stamped = PoseStamped()
    quaternion_angle = tf.transformations.quaternion_from_euler(0, 0, poseTheta[i])
    pose_stamped.pose = Pose( Point(poseX[i], poseY[i], 0.0),  Quaternion(*quaternion_angle) )  
    pose_stamped.header.seq = i
    pose_stamped.header.stamp = tstamp[i]
    pose_stamped.header.frame_id = "/map"
    path.poses.append(pose_stamped)

    #Create pose msg for odometry
    g_truth_pose = create_pose_stamped(poseX[i], poseY[i], poseTheta[i], tstamp[i], i)

    #Create TF data msg
    #g_truth_path_tf = create_TFmsg(poseX[i], poseY[i], 0, poseTheta[i], "/map", "/Path",tstamp[i], i)
 
    #Write data to bag
    #bag.write("/tf", g_truth_path_tf, tstamp[i])
    bag.write("/g_truth", path, tstamp[i])
    bag.write("/g_truth/Pose", g_truth_pose, tstamp[i])
    
bag.close() #export rosbag file to /home/user/.ros 
file.close() #close debugging txt file
 
