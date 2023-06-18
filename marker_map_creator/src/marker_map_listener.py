#!/usr/bin/env python  
import roslib
import rospy
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('marker_map_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('camera/RGB1/Image', 'map',rospy.Time(0))
            print("the trans listened is ", trans, rot)
            p1 = geometry_msgs.msg.PoseStamped()
            p1.header.frame_id = "camera/RGB1/Image"
            p1.pose.position.x = 2.509
            p1.pose.position.y = 0.51
            p1.pose.position.z = 0.63
            p1.pose.orientation.w = 1.0    # Neutral orientation
            p_in_base = listener.transformPose("map", p1)
            print ("Position of the camera/RGB1/Image in the map:", p_in_base)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
