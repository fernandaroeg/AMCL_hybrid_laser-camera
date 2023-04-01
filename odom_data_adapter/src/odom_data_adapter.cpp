#include<ros/ros.h>
#include<std_msgs/Int32.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_broadcaster.h>
#include<string>
#include<math.h>
#include<random>
#include <chrono>
//NODE that suscribes to topic with groundtruth pose data and injects gaussian noise to it, and publishes in odom topic

//GLOBAL VARIABLES
float var_x, var_y, var_theta;
float x, y, theta;
bool  first_pose_received = true;
double two_pi = 2*M_PI;
 
geometry_msgs::Pose pose_now, pose_prev, initial_pose;
geometry_msgs::Pose increment, noise, odom_w_noise;


//FUNCTION DEFINITIONS
geometry_msgs::Pose oplus(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
	geometry_msgs::Pose oplus;
	float angle_sum, angle_wrap;
	
	oplus.position.x     = pose1.position.x + (pose2.position.x * cos(pose1.orientation.w)) - ( pose2.position.y * sin(pose1.orientation.w) );
	oplus.position.y     = pose1.position.y + (pose2.position.x * sin(pose1.orientation.w)) + ( pose2.position.y * cos(pose1.orientation.w) );
	//oplus.orientation.w  = pose1.orientation.w + pose2.orientation.w;
	angle_sum = pose1.orientation.w + pose2.orientation.w;
	angle_wrap = angle_sum - two_pi * floor( angle_sum / two_pi);
	oplus.orientation.w  = angle_wrap; //The sum of the angles is wrapped so that it is inside the range [0,2pi]
	
	return oplus;
}

geometry_msgs::Pose ominus(geometry_msgs::Pose pose1)
{
	geometry_msgs::Pose ominus;
	
	ominus.position.x     = -(pose1.position.x * cos(pose1.orientation.w)) - ( pose1.position.y * sin(pose1.orientation.w) );
	ominus.position.y     =  (pose1.position.x * sin(pose1.orientation.w)) - ( pose1.position.y * cos(pose1.orientation.w) );
	ominus.orientation.w  = -(pose1.orientation.w); 
		
	return ominus;
}
    
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
	x = msg -> pose.position.x;
	y = msg -> pose.position.y;
	tf::Quaternion q( msg->pose.orientation.x,
					  msg->pose.orientation.y,
					  msg->pose.orientation.z,
					  msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	theta = yaw;
}

//MAIN PROGRAM
int main(int argc, char **argv)
{
	//Initialize
	ros::init(argc, argv, "odom_data_adapter");
	ros::NodeHandle nh;
	
	//Declare publishers and suscriber
	ros::Subscriber sub             = nh.subscribe("g_truth/Pose", 1000, poseCallback);
	ros::Publisher  odom_pub        = nh.advertise<nav_msgs::Odometry>("odom", 1000); 
	ros::Publisher  path_pub        = nh.advertise<nav_msgs::Path>("trajectory",1000);
	ros::Publisher  path_pub_gtruth = nh.advertise<nav_msgs::Path>("trajectory_gtruth",1000);
	ros::Publisher  init_pose_pub   = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1000,true); //latch topic
	ros::Publisher  odom_pose_pub   = nh.advertise<geometry_msgs::PoseStamped>("odom_pose",1000); 
	tf::TransformBroadcaster odom_broadcaster;
	
	//Get params from launch file
	nh.getParam("/odom_data_adapter/var_x", var_x);
	nh.getParam("/odom_data_adapter/var_y", var_y);
	nh.getParam("/odom_data_adapter/var_theta", var_theta);
	
	//Get current time in variable
	ros::Time current_time, prev_time;
	
	//Declare path messages
	nav_msgs::Path path, groundtruth_path;
	
	//Node frequency
	ros::Rate r(10.0);
	
	while(nh.ok()){
		ros::spinOnce();  //check for incoming messages
		current_time = ros::Time::now(); //get current time
		
		//Determine current and previous poses
		if (first_pose_received==true)
			{
				pose_prev.position.x     = x; //For the first pose received the value of groundtruth is assigned, so pose_prev = pose_now
				pose_prev.position.y     = y;
				pose_prev.orientation.w  = theta;
				first_pose_received = false; //change flag for subsequent readings
				
				//publish initial pose to the /initialpose topic for amcl
				initial_pose.position.x = x;
				initial_pose.position.y = y;
				geometry_msgs::Quaternion init_pose_quat = tf::createQuaternionMsgFromYaw(theta);
				initial_pose.orientation = init_pose_quat;
				geometry_msgs::PoseWithCovarianceStamped initialpose_msg;
				initialpose_msg.header.stamp = ros::Time::now();
				initialpose_msg.header.frame_id = "/map"; //PENDING verificar
				initialpose_msg.pose.pose.position    = initial_pose.position;
				initialpose_msg.pose.pose.orientation = initial_pose.orientation;
				initialpose_msg.pose.covariance[0] = var_x*2;
				initialpose_msg.pose.covariance[7] = var_y*2;
				initialpose_msg.pose.covariance[14] = 999;
				initialpose_msg.pose.covariance[21] = 999;
				initialpose_msg.pose.covariance[28] = 999;
				initialpose_msg.pose.covariance[35] = var_theta*2;
				init_pose_pub.publish(initialpose_msg);
				prev_time = current_time;
			}
		else
			{		
				pose_prev.position.x     = pose_now.position.x;  //Assign  the value saved in the last cycle of pose w noise added
				pose_prev.position.y     = pose_now.position.y;
				pose_prev.orientation.w  = pose_now.orientation.w;
			}
		
		pose_now.position.x     = x; //ground truth data being received by suscriber callback
		pose_now.position.y     = y;
		pose_now.orientation.w  = theta;
		ROS_INFO("New sequence comming");
		ROS_INFO("Pose prev is x:%f y:%f th:%f", pose_prev.position.x, pose_prev.position.y, pose_prev.orientation.w);
		ROS_INFO("Pose now is x:%f y:%f th:%f", pose_now.position.x, pose_now.position.y, pose_now.orientation.w);		
		
		//****ADD GAUSSIAN NOISE****//
		//1. compute increment
		increment = oplus( ominus(pose_prev), pose_now);
		ROS_INFO("Pose increment is x:%f y:%f th:%f", increment.position.x, increment.position.y, increment.orientation.w);
		
		//2. compute random normal noise
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		std::default_random_engine generator(seed);
		std::normal_distribution<double> distribution(/*mean*/ 0.0, /*std_dev*/ 1.0);
		double random_number = distribution(generator);
		
		noise.position.x    = sqrt(var_x    ) * random_number;
		noise.position.y    = sqrt(var_y    ) * random_number;
		noise.orientation.w = sqrt(var_theta) * random_number;
		ROS_INFO("Pose noise is x:%f y:%f th:%f", noise.position.x, noise.position.y, noise.orientation.w);
		
		//3. add noise to increment
		increment = oplus(increment, noise);
		ROS_INFO("Pose incr w noise x:%f y:%f th:%f", increment.position.x, increment.position.y, increment.orientation.w);
		
		//4. project noise to previous pose
		odom_w_noise = oplus( odom_w_noise, increment);
		ROS_INFO("Odom w noise is x:%f y:%f th:%f", odom_w_noise.position.x, odom_w_noise.position.y, odom_w_noise.orientation.w);
		//****END ADD GAUSSIAN NOISE****//
		
		//5. compute velocities for twist message, velocity in the robot is constant , 0.1m/s, so no need to express covariance in velocity (Twist msg)   
		float dx, dy, dyaw, vx, vy, vth, dt;
		dx   = pose_now.position.x - pose_prev.position.x;
		dy   = pose_now.position.y - pose_prev.position.y;
		dyaw = pose_now.orientation.w - pose_prev.orientation.w;
		dt   = current_time.toSec() - prev_time.toSec();
		
		if (dt == 0)
			{
				vx = 0;
				vy = 0;
				vth= 0;
			}
		else
			{
				vx= dx/dt;
				vy= dy/dt;
				vth = dyaw/dt;
			}
		
		//transform theta to quaternion for odom msg, odom_quat
		geometry_msgs::Quaternion g_truth_quat = tf::createQuaternionMsgFromYaw(theta);
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_w_noise.orientation.w); //angle value stored in .orientation.w variable
		
		//publish odometry msg over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp          = current_time;
		odom.header.frame_id       = "odom";
		odom.child_frame_id        = "base_link";
		odom.pose.pose.position.x  = odom_w_noise.position.x;
		odom.pose.pose.position.y  = odom_w_noise.position.y;
		odom.pose.pose.position.z  = 0.0;
		odom.pose.pose.orientation = odom_quat;
		odom.pose.covariance[0] = var_x;
		odom.pose.covariance[7] = var_y;
		odom.pose.covariance[14] = 999;
		odom.pose.covariance[21] = 999;
		odom.pose.covariance[28] = 999;
		odom.pose.covariance[35] = var_theta;
		odom.twist.twist.linear.x  = vx;
		odom.twist.twist.linear.y  = vy;
		odom.twist.twist.angular.z = vth;

		//publish the msg
		odom_pub.publish(odom);
		
		//publishh transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp            = current_time;
		odom_trans.header.frame_id         = "odom";
		odom_trans.child_frame_id          = "base_link";
		odom_trans.transform.translation.x = odom_w_noise.position.x;
		odom_trans.transform.translation.y = odom_w_noise.position.y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation      = odom_quat;
		//send transform
		odom_broadcaster.sendTransform(odom_trans);
		
		//create path and pose msg w/odom information
		geometry_msgs::PoseStamped odom_pose_stamped;
		odom_pose_stamped.pose.position.x  = odom_w_noise.position.x;
		odom_pose_stamped.pose.position.y  = odom_w_noise.position.y;
		odom_pose_stamped.pose.orientation = odom_quat;
		odom_pose_stamped.header.stamp     = current_time;
		odom_pose_stamped.header.frame_id  = "map";
		path.poses.push_back(odom_pose_stamped);
		path.header.stamp = current_time;
		path.header.frame_id = "map";
		path_pub.publish(path);
		odom_pose_pub.publish(odom_pose_stamped);
		
		//create path msg w/ground truth information
		geometry_msgs::PoseStamped groundtruth_pose_stamped;
		groundtruth_pose_stamped.pose.position.x  = x;
		groundtruth_pose_stamped.pose.position.y  = y;
		groundtruth_pose_stamped.pose.orientation = g_truth_quat;
		groundtruth_pose_stamped.header.stamp     = current_time;
		groundtruth_pose_stamped.header.frame_id  = "map";
		groundtruth_path.poses.push_back(groundtruth_pose_stamped);
		groundtruth_path.header.stamp = current_time;
		groundtruth_path.header.frame_id = "map";
		path_pub_gtruth.publish(groundtruth_path);
				
		r.sleep();
		prev_time = current_time;
	}
	//ros::spin();
	//return 0;
}
