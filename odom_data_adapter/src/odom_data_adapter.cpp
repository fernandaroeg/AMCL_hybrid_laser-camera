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
//#include<random> PENDING

//PARAMETER SETUP
float var_x = 0.00001;  //covariance in x, value taken from turtlebot sim in gazebo
float var_y = 0.00001;  //covariance in y, value taken from turtlebot sim in gazebo
float var_theta = 0.01; //covariance in z/yaw, value taken from turtlebot sim in gazebo
float noise_range_x = 0.01; //1cm expressed in meters
float noise_range_y = 0.01; //1cm expressed in meters
float noise_range_theta = 0.17; //0.1 degreed expressed in radians
	
//GLOBAL VARIABLES
float x;
float y;
float theta;
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

pose add_gaussian_noise(pose *pose_prev, pose *pose_now, pose *var, pose *noise)
{
	pose increment;
	pose increment_w_noise;
	pose pose_w_noise;
	
	//Compute the position increment
	increment.x     = pose_now->x     - pose_prev->x;
	increment.y     = pose_now->y     - pose_prev->y;
	increment.theta = pose_now->theta - pose_prev->theta;
	
	//Create random numbers from normal distribution with range noise PENDING
	
	//Add the gaussian noise to the current position increment
	increment_w_noise.x     = increment.x     + ( sqrt(var->x)     * noise->x);
	increment_w_noise.y     = increment.y     + ( sqrt(var->y )    * noise->y);
	increment_w_noise.theta = increment.theta + ( sqrt(var->theta) * noise->theta);
	// increment_w_noise.x     = increment.x     + ( sqrt(var->x)     * rand_x(gen)     );
	// increment_w_noise.y     = increment.y     + ( sqrt(var->y )    * rand_y(gen)     );
	// increment_w_noise.theta = increment.theta + ( sqrt(var->theta) * rand_theta(gen) );
	
	pose *increment_w_noise_ptr = &increment_w_noise; //we all hate pointers
	
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
	ROS_INFO("%f", theta);
}

//MAIN PROGRAM
int main(int argc, char **argv)
{
	//Initialize
	ros::init(argc, argv, "odom_data_adapter");
	ros::NodeHandle nh;
	
	//Declare publishers and suscriber
	ros::Subscriber sub = nh.subscribe("g_truth/Pose", 1000, poseCallback);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory",50);
	ros::Publisher path_pub_gtruth = nh.advertise<nav_msgs::Path>("trajectory_gtruth",50);
	tf::TransformBroadcaster odom_broadcaster;
	
	//Get current time in variable
	ros::Time current_time;
	current_time = ros::Time::now();
	
	//Declare path messages
	nav_msgs::Path path;
	path.header.stamp = current_time;
	path.header.frame_id = "odom";
	
	nav_msgs::Path groundtruth_path;
	groundtruth_path.header.stamp = current_time;
	groundtruth_path.header.frame_id = "odom";
	
	//Node frequency
	ros::Rate r(1.0);
	
	while(nh.ok()){
		ros::spinOnce();  //check for incoming messages
		
		//Determine current and previous poses
		if (first_pose_received == true)
			{
				pose_prev.x     = 0;
				pose_prev.y     = 0;
				pose_prev.theta = 0;
			}
		else
			{
				//pose_prev.x     = odom_w_noise.x;
				//pose_prev.y     = odom_w_noise.y;
				//pose_prev.theta = odom_w_noise.theta;
				
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
		
		//create path msg w/ground truth information
		geometry_msgs::PoseStamped groundtruth_pose_stamped;
		groundtruth_pose_stamped.pose.position.x  = x;
		groundtruth_pose_stamped.pose.position.y  = y;
		groundtruth_pose_stamped.pose.orientation = g_truth_quat;
		groundtruth_pose_stamped.header.stamp     = current_time;
		groundtruth_pose_stamped.header.frame_id  = "g_truth";
		groundtruth_path.poses.push_back(groundtruth_pose_stamped);
		
		path_pub.publish(path);
		path_pub_gtruth.publish(groundtruth_path);
		
		first_pose_received = false; //change flag for subsequent readings
		
		r.sleep();
	}
	//ros::spin();
	//return 0;
}
