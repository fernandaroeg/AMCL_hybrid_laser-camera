#! /usr/bin/env python
#Ubuntu 16.04, ROS Kinetic, python 2.7
import rospy
import rosbag
from sensor_msgs.msg   import LaserScan
from tf2_msgs.msg      import TFMessage 
from geometry_msgs.msg import TransformStamped
import tf
from tf.transformations import euler_from_quaternion
#Node that reads data published from node pointcloud_to_laserscan and rewrites data with correct topic name and framed to be accepted by the AMCL node

pub_scan = rospy.Publisher('/scan', LaserScan, queue_size = 10)
pub_tf   = rospy.Publisher("/tf", TFMessage, queue_size=10)   
scan_received = LaserScan()    

def create_TFmsg(x, y, z, theta, frame, child_frame):
    trans = TransformStamped()
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

def callback(msg):
    current_time = rospy.Time.now()
    scan_received.header.seq      = msg.header.seq 
    scan_received.header.stamp    = current_time
    scan_received.header.frame_id = '/laser'
    scan_received.angle_min       = msg.angle_min      
    scan_received.angle_max       = msg.angle_max      
    scan_received.angle_increment = msg.angle_increment
    scan_received.time_increment  = msg.time_increment 
    scan_received.scan_time       = msg.scan_time      
    scan_received.range_min       = msg.range_min      
    scan_received.range_max       = msg.range_max      
    scan_received.ranges          = msg.ranges         
    
    pub_scan.publish(scan_received)
    new_tf_msg   = create_TFmsg(0.285,0,1.0450,0,'/base_link','/laser')
    pub_tf.publish(new_tf_msg)

def listener():
    rospy.init_node('depth_to_laser_adapter', anonymous=True)
    sub = rospy.Subscriber('/camera/RGB1/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
 