#include<ros/ros.h>
#include<std_msgs/Int32.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_broadcaster.h>
#include<string>

//PARAMETER SETUP
std::string scenario = "alma";
std::string bag_name = "odometry_data";
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_data_adapter");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("g_truth/Pose", 1000, poseCallback);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory",50);
	tf::TransformBroadcaster odom_broadcaster;
	
	ros::Time current_time;
	current_time = ros::Time::now();
	
	nav_msgs::Path path;
	path.header.stamp = current_time;
	path.header.frame_id = "odom";
	
	ros::Rate r(1.0);
	
	while(nh.ok()){
		ros::spinOnce();  //check for incoming messages
		
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
		//publishh transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		//send transform
		odom_broadcaster.sendTransform(odom_trans);
		//publish odometry msg over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		odom.child_frame_id = "base_link";
		//publish the msg
		odom_pub.publish(odom);
		
		//create path msg w/odom information
		geometry_msgs::PoseStamped odom_pose_stamped;
		odom_pose_stamped.pose.position.x = x + 1;
		odom_pose_stamped.pose.position.y = y;
		odom_pose_stamped.pose.orientation = odom_quat;
		odom_pose_stamped.header.stamp = current_time;
		odom_pose_stamped.header.frame_id = "odom";
		path.poses.push_back(odom_pose_stamped);
		
		path_pub.publish(path);
		
		r.sleep();
	}
	//ros::spin();
	//return 0;
}
