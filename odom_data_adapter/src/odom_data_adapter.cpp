#include<ros/ros.h>
#include<std_msgs/Int32.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_broadcaster.h>
#include<string>
#include<math.h>
#include<random>

//PARAMETER SETUP
float var_x     = 0.001;  //variance in x, experimental values
float var_y     = 0.001;  //variance in y, experimental values
float var_theta = 0.0076; //variance in z, experimental values

	
//GLOBAL VARIABLES
float x;
float y;
float theta;
float x_odom;
float y_odom;
float theta_odom;
bool  first_pose_received = true;

geometry_msgs::Pose pose_now;
geometry_msgs::Pose pose_prev;
geometry_msgs::Pose odom_w_noise;

//FUNCTIONS DEFINITIONS
geometry_msgs::Pose oplus(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
	geometry_msgs::Pose oplus;
	
	oplus.position.x     = pose1.position.x + (pose2.position.x * cos(pose1.orientation.w)) - ( pose2.position.y * sin(pose1.orientation.w) );
	oplus.position.y     = pose1.position.y + (pose2.position.x * sin(pose1.orientation.w)) + ( pose2.position.y * cos(pose1.orientation.w) );
	oplus.orientation.w  = pose1.orientation.w + pose2.orientation.w; //verificar si no hace falta un modulo aqui PENDING
		
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

geometry_msgs::Pose add_gaussian_noise(geometry_msgs::Pose pose_prev, geometry_msgs::Pose pose_now, float var_x, float var_y, float var_theta)
{
	geometry_msgs::Pose increment;
	geometry_msgs::Pose increment_w_noise;
	geometry_msgs::Pose pose_w_noise;
	geometry_msgs::Pose pose_prev_inv;
	
	//Compute the pose increment
	increment = oplus( ominus(pose_prev), pose_now);
	
	//Create random numbers from normal distribution with range noise
	std::default_random_engine generator;
	std::normal_distribution<double> distribution(/*mean*/ 0.0, /*std_dev*/ 1.0);
	double random_number = distribution(generator);
	
	//Add the gaussian noise to the current position increment
	increment_w_noise.position.x     = increment.position.x     + ( sqrt(var_x)     * random_number);
	increment_w_noise.position.y     = increment.position.y     + ( sqrt(var_y )    * random_number);
	increment_w_noise.orientation.w  = increment.orientation.w  + ( sqrt(var_theta) * random_number);
	
	
	//Add error in the increment to the latest pose, this error is in a new ref.frame due to drift
	//To add this error to the latest pose in another ref.frame the rel. tranfsm. oplus is used
	//odom_pose_now_noise = pose_now /oplus increment_w_noise
	pose_w_noise = oplus(pose_prev, increment_w_noise);
	
	return pose_w_noise;
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
	//ROS_INFO("%f", theta);
}

void odom_poseCallback(const geometry_msgs::PoseStamped::ConstPtr &odom_msg) {
	x_odom = odom_msg -> pose.position.x;
	y_odom = odom_msg -> pose.position.y;
	tf::Quaternion q( odom_msg->pose.orientation.x,
					  odom_msg->pose.orientation.y,
					  odom_msg->pose.orientation.z,
					  odom_msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	theta_odom = yaw;
}

//MAIN PROGRAM
int main(int argc, char **argv)
{
	//Initialize
	ros::init(argc, argv, "odom_data_adapter");
	ros::NodeHandle nh;
	
	//Declare publishers and suscriber
	ros::Subscriber sub      = nh.subscribe("g_truth/Pose", 1000, poseCallback);
	ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odom_poseCallback);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000); 
	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory",1000);
	ros::Publisher path_pub_gtruth = nh.advertise<nav_msgs::Path>("trajectory_gtruth",1000);
	tf::TransformBroadcaster odom_broadcaster;
	
	//Get current time in variable
	ros::Time current_time;
	
	//Declare path messages
	nav_msgs::Path path;	
	nav_msgs::Path groundtruth_path;
	
	//Node frequency
	ros::Rate r(10.0);
	
	while(nh.ok()){
		ros::spinOnce();  //check for incoming messages
		current_time = ros::Time::now(); //get current time
		
		//Determine current and previous poses
		if (first_pose_received == true)
			{
				pose_prev.position.x     = x; //For the first pose received the value of groundtruth is assigned, so pose_prev = pose_now
				pose_prev.position.y     = y;
				pose_prev.orientation.w  = theta;
			}
		else
			{	
				pose_prev.position.x     = pose_now.position.x;  //Assign  the value saved in the last cycle
				pose_prev.position.y     = pose_now.position.y;
				pose_prev.orientation.w  = pose_now.orientation.w;
			}
		
		first_pose_received = false; //change flag for subsequent readings
		
		pose_now.position.x     = x; //ground truth data being received by suscriber callback
		pose_now.position.y     = y;
		pose_now.orientation.w  = theta;
			
		//add noise to data
		odom_w_noise = add_gaussian_noise(pose_prev, pose_now, var_x, var_y, var_theta);
		
		//transform theta to quaternion for odom msg, odom_quat
		geometry_msgs::Quaternion g_truth_quat = tf::createQuaternionMsgFromYaw(theta);
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_w_noise.orientation.w); //angle value stored in .orientation.w variable
		
		//publish odometry msg over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp          = current_time;
		odom.header.frame_id       = "odom";
		odom.pose.pose.position.x  = odom_w_noise.position.x;
		odom.pose.pose.position.y  = odom_w_noise.position.y;
		odom.pose.pose.position.z  = 0.0;
		odom.pose.pose.orientation = odom_quat;
		odom.child_frame_id        = "base_link";
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
		
		//create path msg w/odom information
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
	}
	//ros::spin();
	//return 0;
}
