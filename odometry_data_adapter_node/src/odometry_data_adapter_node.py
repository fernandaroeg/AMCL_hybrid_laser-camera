#! /usr/bin/env python
#Ubuntu 16.04, ROS Kinetic, python 2.7
#Odometry adapter node. The laser scan matcher only outputs 2D pose messages that need to be transformed to Odometry msg
#ROS odometry messages are formed by header, pose msg, twist msg and covariance information (gaussian noise in the data)
#pose2d transformed to pose, twist estimated from pose & tstamps, data will have gaussian noise injected to 'simulate' wheel encoder noise
import rospy
import rosbag
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,  Point, Twist, Quaternion, Vector3, TransformStamped
from tf2_msgs.msg import TFMessage 
import tf
import numpy as np

#### 0. Parameter Setup ####
scenario = "alma"
bag_name = 'odometry_data'
var_x = 0.00001 # covariance in x, value taken from turtlebot sim in gazebo
var_y = 0.00001 # covariance in y, value taken from turtlebot sim in gazebo
var_theta = 0.001 # covariance in z/yaw, value taken from turtlebot sim in gazebo
noise_range_x = 0.0#0.01 #1cm expressed in meters
noise_range_y = 0.0#0.01 #1cm expressed in meters
noise_range_theta = 0.0#0.0017 #0.1 degreed expressed in radians

#### 1.a Read bag file with 2D pose data created by laser-scan-matcher ####
bag = rosbag.Bag('/home/fer/Desktop/catkin_ws/src/ROS_AMCL_Hybrid_Localization/odometry_data_adapter_node/data/alma_laser_scan_output.bag')
topic = '/pose2D'
pose2D_list = []
topic_list = []
tstamp_list = []
for topic, msg, t in bag.read_messages(topics=topic):
    pose2D_list.append(msg)
    topic_list.append(topic)
    tstamp_list.append(t)

#### 1.b Read data from ground truth with 2D pose ####
#Archivo del data set que expresa x (metros), y (metros), theta (radianes)
gtruth_file="/home/fer/Desktop/catkin_ws/src/ROS_AMCL_Hybrid_Localization/odometry_data_adapter_node/data/alma_log_estimated_path.txt"
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
        
#### 1.b.1 Logic to put timestamp data in list transformed from TTimeStamp format to unix epoch ####
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

#### 2. Functions to add gaussian noise to data ####
def ominus(x, y, theta):
    
    x_inv =  -x*np.cos(theta)  -y*np.sin(theta)
    y_inv =   x*np.sin(theta)   -y*np.cos(theta) 
    theta_inv = -theta
    
    return x_inv, y_inv, theta_inv
    
def oplus(x1, y1, theta1, x2, y2, theta2):
   
   x_oplus = x1 + x2*np.cos(theta1) - y2*np.sin(theta1)
   y_oplus = y1 + x2*np.sin(theta1) + y2*np.cos(theta1)
   theta_oplus = (theta1 + theta2) % (2*np.pi)
  
   return x_oplus, y_oplus, theta_oplus

def add_gaussian_nose(prev_x, prev_y, prev_theta, x,y,theta, var_x, var_y, var_theta, noise_range_x, noise_range_y, noise_range_theta):
                                           #(prev_pose, current_pose, var, noise)
    #Compute the real actual position increment
    #inc_odom_real = odom_pose_previous /ominus  odom_pose_now
    odom_now_x, odom_now_y, odom_now_theta = ominus(x , y , theta)
    
    inc_odom_real_x = prev_x + odom_now_x
    inc_odom_real_y = prev_y + odom_now_y
    inc_odom_real_theta = prev_theta + odom_now_theta
    
    #Add the gaussian noise to the current position increment
    inc_odom_noise_x        = inc_odom_real_x + np.sqrt(var_x) * (np.random.normal() * noise_range_x)
    inc_odom_noise_y        = inc_odom_real_y + np.sqrt(var_y) * (np.random.normal() * noise_range_y)
    inc_odom_noise_theta = inc_odom_real_theta + np.sqrt(var_theta) * (np.random.normal() * noise_range_theta)
    
    #Add error in the increment to the latest pose
    #odom_pose_now_noise = odom_pose_now /oplus inc_odom_noise
    odom_now_noise_x, odom_now_noise_y, odom_now_noise_theta = oplus(x, y, theta, inc_odom_noise_x, inc_odom_noise_y, inc_odom_noise_theta)
    
    return odom_now_noise_x, odom_now_noise_y, odom_now_noise_theta
    
 #### 3. Function to create TF odometry data
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
    
##### 4. Open bag file to write Odometry data in it ####
#List names with data to write from groundtruth: poseX, poseY, poseTheta, tstamp
bag = rosbag.Bag(bag_name+'_'+scenario+'.bag', 'w')
file = open('odom_debug.txt', 'w') #create debugging txt file
    

odometry_msg = Odometry()
for i in range(0,len(poseX)):
    #Header msg data
    odometry_msg.header.seq = i
    odometry_msg.header.frame_id = "/odom"
    odometry_msg.child_frame_id    = "/base_link"
    odometry_msg.header.stamp = tstamp[i]
    
    #Add noise to data
    if i == 0:#no previous odom reading
        prev_x = 0
        prev_y = 0
        prev_theta = 0
        prev_t = 0
    else:
        prev_x = poseX[i-1]
        prev_y = poseY[i-1]
        prev_theta = poseTheta[i-1]
        prev_t = tstamp[i-1].to_nsec()
        
    current_x = poseX[i]
    current_y = poseY[i]
    current_theta = poseTheta[i]
    
    odom_now_noise_x, odom_now_noise_y, odom_now_noise_theta = add_gaussian_nose(prev_x, prev_y, prev_theta, current_x, current_y, current_theta, var_x, var_y, var_theta, noise_range_x, noise_range_y, noise_range_theta)
    
    #Pose with covariance msg data, movement is in 2D so only x,y and yaw data available
    quaternion_angle = tf.transformations.quaternion_from_euler(0, 0, odom_now_noise_theta)
    odometry_msg.pose.pose = Pose( Point(odom_now_noise_x, odom_now_noise_y, 0.0),  Quaternion(*quaternion_angle) )  #investigar q hace ese apuntador
    odometry_msg.pose.covariance[0]  = var_x # covariance in x, value taken from turtlebot sim in gazebo
    odometry_msg.pose.covariance[7]  = var_y # covariance in y, value taken from turtlebot sim in gazebo
    odometry_msg.pose.covariance[14]  = 99999 #covariance in z, not used
    odometry_msg.pose.covariance[21]  = 99999 #covariance in pitch, not used
    odometry_msg.pose.covariance[28]  = 99999 #covariance in roll, not used 
    odometry_msg.pose.covariance[35] = var_theta # covariance in z/yaw, value taken from turtlebot sim in gazebo
    
    #Export generated odom for debugging purposes
    file.write(str(odom_now_noise_x))
    file.write(' ')
    file.write(str(odom_now_noise_y))
    file.write('\n')
    
    #Twist with covariance msg data, velocity in the robot is constant , 0.1m/s, so no need to express covariance in velocity (Twist msg)       
    dx =      current_x - prev_x
    dy =      current_y - prev_y
    dyaw = current_theta - prev_theta
    dt =       tstamp[i].to_nsec() - prev_t
    
    if dt == 0:
        vx = 0
        vy = 0
        vth= 0
    else:
        vx= dx/dt
        vy= dy/dt
        vth = dyaw/dt
        print vx, vy, vth
    odometry_msg.twist.twist = Twist(Vector3(vx, vy, 0),  Vector3(0, 0, vth) )
    
    #Odometry source must publish info. about the TF frame it manages, call function to generate TF odom data msg
    odom_base_link_tf = create_TFmsg(0,0,0,0, "/odom", "/base_link",tstamp[i], i)
    #Create additional TF msgs
    map_odom_tf          = create_TFmsg(odom_now_noise_x, odom_now_noise_y, -0.1, odom_now_noise_theta, "/map", "/odom",tstamp[i], i)
 #   world_map_tf          = create_TFmsg(0.000602, 0.002559, 0.0, -0.002202, "/world", "/map",tstamp[i], i)
    world_map_tf          = create_TFmsg(0.0, 0.0, 0.0, 0.0, "/world", "/map",tstamp[i], i)
 
    #Write data to bag
    bag.write("/odom", odometry_msg, tstamp[i])
    bag.write("/tf", odom_base_link_tf, tstamp[i])
    bag.write("/tf", map_odom_tf, tstamp[i])
    bag.write("/tf", world_map_tf, tstamp[i])
    
bag.close() #export rosbag file to /home/user/.ros 
file.close() #close debugging txt file
 
# #####4. Open bag file to write Odometry data in it --Scan-matcher data
# scenario = "alma" #!!!!! get this data from launch file
# bag = rosbag.Bag('odometry_data_'+scenario+'.bag', 'w')

# odometry_msg = Odometry()
# for i in range(0,len(pose2D_list)):
    # #Header msg data
    # odometry_msg.header.seq = i
    # odometry_msg.header.frame_id = "/odom"
    # odometry_msg.child_frame_id    = "/base_link"
    # odometry_msg.header.stamp = tstamp_list[i]
    
    # #Pose with covariance msg data, movement is in 2D so only x,y and yaw data available
    # quaternion_angle = tf.transformations.quaternion_from_euler(0, 0, pose2D_list[i].theta)
    # odometry_msg.pose.pose = Pose( Point(pose2D_list[i].x, pose2D_list[i].y, 0.0),  Quaternion(*quaternion_angle) )  #investigar q hace ese apuntador
    # odometry_msg.pose.covariance[0]  = 0.00001 # covariance in x, value taken from turtlebot sim in gazebo
    # odometry_msg.pose.covariance[7]  = 0.00001 # covariance in y, value taken from turtlebot sim in gazebo
    # odometry_msg.pose.covariance[14]  = 99999 #covariance in z, not used
    # odometry_msg.pose.covariance[21]  = 99999 #covariance in pitch, not used
    # odometry_msg.pose.covariance[28]  = 99999 #covariance in roll, not used 
    # odometry_msg.pose.covariance[35] = 0.001 # covariance in z/yaw, value taken from turtlebot sim in gazebo
    
    # #Twist with covariance msg data, velocity in the robot is constant , 0.1m/s, so no need to express covariance in velocity (Twist msg)
    # if i == 0:
        # prev_x = 0
        # prev_y = 0
        # prev_yaw = 0
        # prev_t = 0
    # else:
        # prev_x = pose2D_list[i-1].x
        # prev_y = pose2D_list[i-1].y
        # prev_yaw = pose2D_list[i-1].theta
        # prev_t = tstamp_list[i-1].to_nsec()
        
    # dx =      pose2D_list[i].x - prev_x
    # dy =      pose2D_list[i].y - prev_y
    # dyaw = pose2D_list[i].theta - prev_yaw #verificar si esta bien trabajar aqui con angulo en grados, radianes, quaternios no??
    # #dquaternion_angle = tf.transformations.quaternion_from_euler(0, 0, dyaw)
    # #dyaw = dquaternion_angle
    # dt =       tstamp_list[i].to_nsec() - prev_t
    
    # if dt == 0:
        # vx = 0
        # vy = 0
        # vth= 0
    # else:
        # vx= dx/dt
        # vy= dy/dt
        # vth = dyaw/dt
    # odometry_msg.twist.twist = Twist(Vector3(vx, vy, 0),  Vector3(0, 0, vth) )
    
    # #Odometry source must publish info. about the TF frame it manages, call function to generate TF odom data msg
    # odom_tf_data = create_TFmsg(pose2D_list[i].x, pose2D_list[i].y,pose2D_list[i].theta,tstamp_list[i])
 
    # #Write data to bag
    # bag.write("/odom", odometry_msg, tstamp_list[i])
    # bag.write("/tf", odom_tf_data, tstamp_list[i])
    
# bag.close() #export rosbag file to /home/user/.ros 