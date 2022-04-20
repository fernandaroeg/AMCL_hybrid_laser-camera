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
float var_x = 0.00001;  //covariance in x, value taken from turtlebot sim in gazebo
float var_y = 0.00001;  //covariance in y, value taken from turtlebot sim in gazebo
float var_theta = 0.01; //covariance in z/yaw, value taken from turtlebot sim in gazebo
float noise_range_x = 0.01; //1cm expressed in meters
float noise_range_y = 0.01; //1cm expressed in meters
float noise_range_theta = 0.17; //0.1 degreed expressed in radians

// float var_x = 0.01;  //covariance in x
// float var_y = 0.01;  //covariance in y
// float var_theta = 0.00028; //covariance in z/yaw
// float noise_range_x = 0.1; //10cm expressed in meters
// float noise_range_y = 0.1; //10cm expressed in meters
// float noise_range_theta = 0.017; //1 degreed expressed in radians
	
//GLOBAL VARIABLES
float x;
float y;
float theta;
float x_odom;
float y_odom;
float theta_odom;
bool first_pose_received = true;

//STRUCTURES
struct pose{float x;
			float y;
			float theta;
			};

struct pose var          = {var_x, var_y, var_theta};
struct pose noise        = {noise_range_x, noise_range_y, noise_range_theta};
struct pose pose_now;
struct pose pose_prev;
struct pose odom_w_noise;
pose *var_ptr          = &var;         
pose *noise_ptr        = &noise;        
pose *pose_now_ptr     = &pose_now;
pose *pose_prev_ptr    = &pose_prev;
pose *odom_w_noise_ptr = &odom_w_noise;

//FUNCTIONS DEFINITIONS
pose oplus(pose *pose1, pose *pose2)
{
	pose oplus;
	
	oplus.x     = pose1->x + (pose2->x * cos(pose1->theta)) - ( pose2->y * sin(pose1->theta) );
	oplus.y     = pose1->y + (pose2->x * sin(pose1->theta)) + ( pose2->y * cos(pose1->theta) );
	oplus.theta = pose1->theta + pose2->theta; //verificar si no hace falta un modulo aqui PENDING
		
	return oplus;
}

pose ominus(pose *pose1)
{
	pose ominus;
	
	ominus.x     = -(pose1->x * cos(pose1->theta)) - ( pose1->y * sin(pose1->theta) );
	ominus.y     =  (pose1->x * sin(pose1->theta)) - ( pose1->y * cos(pose1->theta) );
	ominus.theta = -(pose1->theta); 
		
	return ominus;
}

pose add_gaussian_noise(pose *pose_prev, pose *pose_now, pose *var, pose *noise)
{
	pose increment;
	pose increment_w_noise;
	pose pose_w_noise;
	pose pose_prev_inv;
	
	//Compute the position increment
	// pose_prev_inv = ominus(pose_prev);
	// increment.x     = pose_now->x     - pose_prev_inv.x;
	// increment.y     = pose_now->y     - pose_prev_inv.y;
	// increment.theta = pose_now->theta - pose_prev_inv.theta;
	
	increment.x     = pose_now->x     - pose_prev->x;
	increment.y     = pose_now->y     - pose_prev->y;
	increment.theta = pose_now->theta - pose_prev->theta;
	
	//Create random numbers from normal distribution with range noise
	std::default_random_engine generator;
	std::normal_distribution<double> distribution(/*mean*/ 0.0, /*std_dev*/ 1.0);
	double random_number = distribution(generator);
	
	//Add the gaussian noise to the current position increment
	increment_w_noise.x     = increment.x     + ( sqrt(var->x)     * random_number);
	increment_w_noise.y     = increment.y     + ( sqrt(var->y )    * random_number);
	increment_w_noise.theta = increment.theta + ( sqrt(var->theta) * random_number);
	
	pose *increment_w_noise_ptr = &increment_w_noise;
	
    //Add error in the increment to the latest pose, this error is in a new ref.frame due to drift
    //To add this error to the latest pose in another ref.frame the rel. tranfsm. oplus is used
    //odom_pose_now_noise = pose_now /oplus increment_w_noise
	pose_w_noise = oplus(pose_now, increment_w_noise_ptr);
	
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
				pose_prev.x     = 0;
				pose_prev.y     = 0;
				pose_prev.theta = 0;
			}
		else
			{
				// pose_prev.x     = x_odom; //previous pose published in odom topic w/noise injected
				// pose_prev.y     = y_odom;
				// pose_prev.theta = theta_odom;
				
				pose_prev.x     = pose_now.x;
				pose_prev.y     = pose_now.y;
				pose_prev.theta = pose_now.theta;
			}
		
		pose_now.x     = x; //ground truth data being received by suscriber callback
		pose_now.y     = y;
		pose_now.theta = theta;
			
		//add noise to data
		odom_w_noise = add_gaussian_noise(pose_prev_ptr, pose_now_ptr, var_ptr, noise_ptr);
		
		//transform theta to quaternion for odom msg, odom_quat
		geometry_msgs::Quaternion g_truth_quat = tf::createQuaternionMsgFromYaw(theta);
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_w_noise.theta);
		
		//publish odometry msg over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp          = current_time;
		odom.header.frame_id       = "odom";
		odom.pose.pose.position.x  = odom_w_noise.x;
		odom.pose.pose.position.y  = odom_w_noise.y;
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
		odom_trans.transform.translation.x = odom_w_noise.x;
		odom_trans.transform.translation.y = odom_w_noise.y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation      = odom_quat;
		//send transform
		odom_broadcaster.sendTransform(odom_trans);
		
		//create path msg w/odom information
		geometry_msgs::PoseStamped odom_pose_stamped;
		odom_pose_stamped.pose.position.x  = odom_w_noise.x;
		odom_pose_stamped.pose.position.y  = odom_w_noise.y;
		odom_pose_stamped.pose.orientation = odom_quat;
		odom_pose_stamped.header.stamp     = current_time;
		odom_pose_stamped.header.frame_id  = "odom";
		path.poses.push_back(odom_pose_stamped);
		path.header.stamp = current_time;
		path.header.frame_id = "odom";
		path_pub.publish(path);
		
		//create path msg w/ground truth information
		geometry_msgs::PoseStamped groundtruth_pose_stamped;
		groundtruth_pose_stamped.pose.position.x  = x;
		groundtruth_pose_stamped.pose.position.y  = y;
		groundtruth_pose_stamped.pose.orientation = g_truth_quat;
		groundtruth_pose_stamped.header.stamp     = current_time;
		groundtruth_pose_stamped.header.frame_id  = "g_truth";
		groundtruth_path.poses.push_back(groundtruth_pose_stamped);
		groundtruth_path.header.stamp = current_time;
		groundtruth_path.header.frame_id = "odom";
		path_pub_gtruth.publish(groundtruth_path);
		
		first_pose_received = false; //change flag for subsequent readings
		
		r.sleep();
	}
	//ros::spin();
	//return 0;
}
