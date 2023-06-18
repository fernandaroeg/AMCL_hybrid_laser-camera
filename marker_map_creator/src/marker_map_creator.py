#!/usr/bin/env python
import rospy
import roslib
import rosbag
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
import tf2_ros
import tf 
import tf2_geometry_msgs 
import geometry_msgs.msg
from std_msgs.msg import ColorRGBA
import os
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#Camera calibration parameters, taken from dataset
cx = 157.3245865
cy = 120.0802295
fx = 286.441384
fy = 271.36999

def sortCorners(img, corners, markerID):
    sorted_corner = [list(), list(), list(), list()]
    corner_colors          = ((255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255)) #BGR coners
    #find rectangle centroid, (it can be an irregular rectangle so we want the centroid instead of the center) https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html
    M = cv2.moments(corners)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    w_center = cx
    h_center = cy                 
           
    #clasify corners according to their position compared to the centroid point
    for corner in corners:
        if corner[0][0] < w_center and corner[0][1] < h_center:   #1er cuadrante, arriba izq, azul
            sorted_corner[0] = corner[0]
            corners_img = cv2.circle(img, (corner[0][0], corner[0][1]), 5, corner_colors[0], -1) 
        elif corner[0][0] > w_center and corner[0][1] < h_center: #2do cuadrante, arriba der, verde
            sorted_corner[1] = corner[0]
            corners_img = cv2.circle(img, (corner[0][0], corner[0][1]), 5, corner_colors[1], -1)
        elif corner[0][0] > w_center and corner[0][1] > h_center: #3er cuadrante, abajo der, rojo
            sorted_corner[2] = corner[0]
            corners_img = cv2.circle(img, (corner[0][0], corner[0][1]), 5, corner_colors[2], -1)
        elif corner[0][0] < w_center and corner[0][1] > h_center: #4to cuadrante, abajo izq, amarillo
            sorted_corner[3] = corner[0]
            corners_img = cv2.circle(img, (corner[0][0], corner[0][1]), 5, corner_colors[3], -1)
        else:
            print("Error of Cuadrant or a corner equal to centroid point-irregular polygons is best to avoid")
            return 0
    
    text = str("ID:" + str(markerID))
    corners_img = cv2.putText(img, text, (w_center, h_center), 2, 1, (255, 0, 0), 1, lineType=cv2.LINE_AA)
    corners_img = cv2.drawContours(img, [corners], -1, (0, 255, 255), 1, cv2.LINE_AA) #draw rectangles to be debuged
    timedate = time.strftime("%Y%m%d-%H%M%S") 
    path = "/home/fer/catkin_ws/src/amcl_hybrid/marker_map_creator/1_input_data/"
    rectangle_export = cv2.imwrite(path+'corners_img_'+str(timedate)+'.bmp',corners_img)
    
    return sorted_corner, corners_img

def pixel_to_point_in_space(pixel, pt_x, pt_y, cx, cy, fx, fy):
    #Function to obtain x,y,z position of pixels in space. Equations taken from dataset webpage    
    h = pt_y
    w = pt_x
    depth = pixel * (1/278.905687) # read each single pixel in image
    x = depth
    z = (cx- h) * x/fx 
    y = (cy- w) * x/fy 
    point = [x*10, y*10, z*10]
    return point

#1. Parameters SETUP
corner_colors          = ((255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255)) #BGR coners
input_data_folder           =  rospy.get_param('marker_map_creator/input_data_folder')#seq, tstamp, points 
outpu_map_path            =  rospy.get_param('marker_map_creator/outpu_map_path')
input_gtruth_bag            =  rospy.get_param('marker_map_creator/input_gtruth_bag')
input_depth_bag            =  rospy.get_param('marker_map_creator/input_depth_bag')
input_depth_img_topic =  rospy.get_param('marker_map_creator/input_depth_img_topic')
input_tf_from_cam         =  rospy.get_param('marker_map_creator/input_tf_from_cam')
bridge = CvBridge()

#2. Read data from text file
files = os.listdir(input_data_folder)
filenames = []
markers_data = []
for file in files:
    if file.endswith( '.txt'):
        filenames.append(file) #append in list all txt files located in the given path
num_files = len(filenames)
print( "num_files is: ", int(num_files))
for file in range(num_files):
    with open (input_data_folder+filenames[file], 'r') as myfile:
        data = myfile.readlines()
        seq      = int(data[0].translate({ord(i): None for i in '\n'}))
        tstamp = data[1].translate({ord(i): None for i in '\n'})
        tstamp = rospy.Time(int( tstamp[:10]), int(tstamp[10:]) )
        pt1 = data[2].translate({ord(i): None for i in '[]'}).split() #get point data, remove unwantes chars, and split
        pt1 = list(map(int, pt1))
        pt2 = data[4].translate({ord(i): None for i in '[]'}).split()
        pt2 = list(map(int, pt2))
        pt3 = data[6].translate({ord(i): None for i in '[]'}).split()
        pt3 = list(map(int, pt3))
        pt4 = data[8].translate({ord(i): None for i in '[]'}).split()
        pt4 = list(map(int, pt4))
        marker = {'seq':seq, 'tstamp':tstamp, 'pt1':pt1, 'pt2':pt2, 'pt3':pt3, 'pt4':pt4 }
        markers_data.append(marker)
        print ("seq is ", seq, "\ntstamp is ", tstamp, "\n pt1 is ", pt1, "\n pt2 is ", pt2, "\n pt3 is ", pt3, "\n pt4 is ", pt4)

#3. Extract depth img from bagfile
depth_bag = rosbag.Bag(input_data_folder+input_depth_bag)
for i in range(0,len(markers_data)):
    for topic, msg, t in depth_bag.read_messages(topics=[input_depth_img_topic]):
        if msg.header.stamp == markers_data[i]['tstamp']:
            print("Tstamp marker: ", markers_data[i]['tstamp'], "coincide con img_depth: ",  msg.header.stamp)
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8") #convert ros img msg to cv_img

            #4 Draw square and points in the img, export image 
            pt1 = markers_data[i]['pt1']
            pt2 = markers_data[i]['pt2'] 
            pt3 = markers_data[i]['pt3']
            pt4 = markers_data[i]['pt4']               
            corners = np.array([pt1, pt2, pt3, pt4], np.int32)
            corners = corners.reshape((-1,1,2))
            #square
            cv2.polylines(cv_image,[corners],True,(255,0,0)) #blue bgr
            #corner points 
            cv2.circle(cv_image, (pt1[0], pt1[1]), radius=0, color=(0, 0, 255), thickness=5) #red
            cv2.circle(cv_image, (pt2[0], pt2[1]), radius=0, color=(0, 255, 0), thickness=5) #green
            cv2.circle(cv_image, (pt3[0], pt3[1]), radius=0, color=(255, 0, 0), thickness=5) #blue
            cv2.circle(cv_image, (pt4[0], pt4[1]), radius=0, color=(0, 255, 255), thickness=5) #yellow
            
            path = "/home/fer/catkin_ws/src/amcl_hybrid/marker_map_creator/1_input_data/"
            #rectangle_export = cv2.imwrite(path+'corners_img_'+str(i)+'.bmp',cv_image)
            
            #5 Get pixel values and compute position in space XYZ  
            pt1_pixel = cv_image[pt1[0], pt1[1]]
            print ("pt1[0] is  ",pt1[0], "\n pt1[1] is ", pt1[1])
            print("el valor del pt1_pixel es ", pt1_pixel[0])           
            pt1_xyz = pixel_to_point_in_space(pt1_pixel, pt1[0], pt1[1], cx, cy, fx, fy)
            print("the xyz points for pt1 are ", pt1_xyz)
            
            pt2_pixel = cv_image[pt2[0], pt2[1]]
            print("el valor del pt2_pixel es ", pt2_pixel[0])
            pt2_xyz = pixel_to_point_in_space(pt2_pixel, pt2[0], pt2[1], cx, cy, fx, fy)
            print("the xyz points for pt2 are ", pt2_xyz)
            
            pt3_pixel = cv_image[pt3[0], pt3[1]]
            print("el valor del pt3_pixel es ", pt3_pixel[0])
            pt3_xyz = pixel_to_point_in_space(pt3_pixel, pt3[0], pt3[1], cx, cy, fx, fy)
            print("the xyz points for pt3 are ", pt3_xyz)
            
            pt4_pixel = cv_image[pt4[0], pt4[1]]
            print("el valor del pt4_pixel es ", pt4_pixel[0])
            print("el valor de seq es ", markers_data[i]['seq'])
            pt4_xyz = pixel_to_point_in_space(pt4_pixel, pt4[0], pt4[1], cx, cy, fx, fy)
            print("the xyz points for pt4 are ", pt4_xyz)

            #6. Transform points to world frame 
            gtruth_bag = rosbag.Bag(input_data_folder+input_gtruth_bag)
            for topic, gt_msg, t in gtruth_bag.read_messages(topics=["tf"]):
                tf_msg = gt_msg.transforms
                #6.1 Find the closest tstamp in the gtruth data to the tstamp when the img was taken
                if tf_msg[0].header.stamp > markers_data[i]['tstamp']: 
                    print("found gtruth tstamp muy cercano: ", tf_msg[0].header.stamp, "al tstamp de img_depth: ", markers_data[i]['tstamp'])
                    #6.2 Extract the pose of the base_link in that tstamp
                    gtruth_transl = tf_msg[0].transform.translation
                    rotation_list  = [tf_msg[0].transform.rotation.x, tf_msg[0].transform.rotation.y, tf_msg[0].transform.rotation.z, tf_msg[0].transform.rotation.w]
                    gtruth_rot      = euler_from_quaternion(rotation_list) #roll, pitch, yaw
                    print("the pose of the base_link in this moment is\ntranslation:\n", gtruth_transl, "\nrotation:\n", gtruth_rot)
                    
                    input_tf_from_cam_transl = [0.271, -0.031, 1.045]
                    input_tf_from_cam_rot      = [90, 0, -45] #roll, pitch, yaw
                    print("the pose of the RGBD1 in this moment is\ntranslation:\n", input_tf_from_cam_transl, "\nrotation:\n", input_tf_from_cam_rot)
                    
                    #6.3 Compute the transform to world points for the 4pts   !!!!                
                    #7. Convert point to marker msg  !!!!!!
                    break
    break #TODO borrar este solo es pa debugear
    
            #7. Sort corners and export data in yaml file 
    
depth_bag.close()
gtruth_bag.close()

def create_marker(x,y,z, t, frame, color):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = t
    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 2
    marker.id = 0
    # Set the scale of the marker
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    # Set the color
    marker.color = color
    # Set the pose of the marker, primero convierto y luego paso aquí los datos convertidos 
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0
    return marker 
    
def create_marker_text(x,y,z, t, frame, color, text):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = t
    marker.type=Marker.TEXT_VIEW_FACING
    marker.id=0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color = color
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0
    marker.text=text
    return marker

def create_static_tf(x,y,z,yaw,time,frame, child_frame):
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = time
    static_transformStamped.header.frame_id = frame
    static_transformStamped.child_frame_id = child_frame
    static_transformStamped.transform.translation.x = x
    static_transformStamped.transform.translation.y = y
    static_transformStamped.transform.translation.z = z
    quat = tf.transformations.quaternion_from_euler(0,0,yaw)
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w= quat[3]
    return static_transformStamped

 
def main(args):
    rospy.init_node('marker_map_creator', anonymous=True)
    
    marker1_pub   = rospy.Publisher("/visualization_marker1", Marker, queue_size = 2)
    marker2_pub   = rospy.Publisher("/visualization_marker2", Marker, queue_size = 2)
    marker3_pub   = rospy.Publisher("/visualization_marker3", Marker, queue_size = 2)
    marker4_pub   = rospy.Publisher("/visualization_marker4", Marker, queue_size = 2)
    markerID_pub = rospy.Publisher("/visualization_markerID", Marker, queue_size = 2)
    broadcaster1 = tf2_ros.StaticTransformBroadcaster()
    broadcaster2 = tf2_ros.StaticTransformBroadcaster()
    broadcaster3 = tf2_ros.StaticTransformBroadcaster()
    
    rate = rospy.Rate(1) # 3 Hz
    time = rospy.Time.now()
    
    pt1_pub   = create_marker(2.509,0.51,0.63, time, "camera/RGB1/Image", ColorRGBA(1, 0, 0, 0.5)) #red
    pt2_pub   = create_marker(2.509,0.52,0.24, time, "camera/RGB1/Image", ColorRGBA(0, 1, 0, 0.5))#green
    pt3_pub   = create_marker(2.509,0.19,0.23, time, "camera/RGB1/Image", ColorRGBA(0, 0, 1, 0.5))#blue
    pt4_pub   = create_marker(2.550,0.18,0.63, time, "camera/RGB1/Image", ColorRGBA(1, 1, 0, 0.5))#yellow
    ptID_pub = create_marker_text(2.550,0.45,0.40, time, "camera/RGB1/Image", ColorRGBA(1, 1, 0, 0.5), "ID Prueba")#yellow
    tf_rgb1   = create_static_tf(0.271,-0.031,1.045,-45, time, "base_link", "camera/RGB1/Image")
    tf_gtruth = create_static_tf(-0.84,6.77,0,1.647,time,"map", "base_link")
    
    #Define pose msg for pt1 to do the frame transformation 
    pt1_world_pose = geometry_msgs.msg.PoseStamped()
    pt1_world_pose.header.frame_id = "camera/RGB1/Image"
    pt1_world_pose.pose.position = (2.509,0.51,0.63)
    rot = quaternion_from_euler(0,0,0)
    pt1_world_pose.pose.orientation = rot

    while not rospy.is_shutdown():
        broadcaster1.sendTransform(tf_rgb1)
        broadcaster2.sendTransform(tf_gtruth)
        marker1_pub.publish(pt1_pub)  
        marker2_pub.publish(pt2_pub)  
        marker3_pub.publish(pt3_pub)  
        marker4_pub.publish(pt4_pub)  
        markerID_pub.publish(ptID_pub)      
        rate.sleep()


if __name__ == "__main__":
    main(sys.argv)
