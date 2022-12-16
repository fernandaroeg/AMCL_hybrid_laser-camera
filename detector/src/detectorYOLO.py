#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from detector.msg import messagedet, pixels_cloud, pixels_corners, num_markers, num_points_cloud
from detector.msg import marker as msg_marker
from geometry_msgs.msg import Point32
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image
import os
import time


class detector:
    def __init__(self, numImages):
        #ROS configuration
        self.bridge = CvBridge()
        self.pub_marker = rospy.Publisher('detected_markers', messagedet, queue_size=1)
        self.pub_projection = rospy.Publisher('detector_img', Image, queue_size=1)
        self.pub_num_marker = rospy.Publisher('num_detecte_markers', num_markers, queue_size=10)
        self.pub_num_points_cloud = rospy.Publisher('num_points_cloud', num_points_cloud, queue_size=20)

        #General detector parameters
        self.numMarkers = numImages
        self.corner_colors = ((255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255)) ## BGR coners
        #Contour detection parameters
        self.thresh_binary  = cv2.THRESH_BINARY_INV # cv2.THRESH_BINARY_INV busca objetos oscuros con fondo claro y cv2.THRESH_BINARY busca objetos claros con fondo oscuro
        #Rectangle detection parameters
        self.min_w_h_image = 10 # minimun width and height image
        #SIFT parameters
        self.minNumMatches = 15
        
        #INICIALIZAR SIFT
        self.detector = cv2.xfeatures2d.SIFT_create() #PENDING cambio importante a registrar en memoria sift feats en vez de orb
        self.bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True) # crea BFMatcher object feature matcher, con normalizacion 1/2 y el crosscheck pending...
        
        #CREAR LISTA DE MARCADORES A BUSCAR
        self.markers = (list(), list(), list()) #se crea lista de 3D para almacenar img, keypoints, descriptors de cada uno de los marcadores 
        for i in range(0, self.numMarkers):
            marker = str("marker" + str(i))
            path = str("/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/detector/markers_alma/"+marker+".png")  #PENDING modificar para tomar path desde launch file  
            print(path)
            img = cv2.imread(path, 0)
            self.markers[0].append(img)
            kp, des = self.detector.detectAndCompute(img, None)
            self.markers[1].append(kp)
            self.markers[2].append(des)   
        print("total images in folder: " + str(len(self.markers[0])))
        
        #START detector process when img is received in topic
        self.image_sub = rospy.Subscriber("/camera/RGB1/Image", Image, self.callback)
        

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            #1.Subscribe to YOLO detection topic and wait for msg
            #yolo_msg = subscribe function
            
            #3. Compare received yolo detecton to stored markers
            if yolo_msg is not None:
                markerID = self.detectID(yolo_msg)  #PENDING cambiar esto a que tenga la imagen original de entrada
                #4. If marker was found sort corners
                corners = self.sortCorners(cv_image, rectangle_crop_points, rect_w_center, rect_h_center) #DeBUGING dentro de esta img publicar img c cuadro detectado
                                                                                                          #PENDING adaptar a partir de este punto la salida del yolo, subscribe to topic
                                                                                                          #objetivo: hacer que yolo mande msj de id en topic
                #5. Publish Marker Msg in TOPIC
                msg_marker = messagedet() #PENDING clean, meter todo esto a una funcion
                msg_marker.DetectedMarkers.append(self.makeMsgMarker(markerID, corners))   
                msg_marker.header.seq = rospy.Duration()
                msg_marker.header.stamp = rospy.Time.now()
                msg_marker.header.frame_id = "camera_link"
                self.pub_marker.publish(msg_marker)
                n_markers = num_markers()
                n_markers.header.stamp = rospy.Time.now()
                n_markers.number = UInt8(len(msg_marker.DetectedMarkers))
                self.pub_num_marker.publish(n_markers)
                print("publico -> " + str(len(msg_marker.DetectedMarkers)))
                print(msg_marker)
                    
            ##6. MOSTRAR Y PUBLICAR IMGS DEL PROCESO
            visualizar = np.concatenate((yolo_img, detection_img), axis=1)
            detector_img = self.bridge.cv2_to_imgmsg(visualizar, encoding="bgr8")
            self.pub_projection.publish(detector_img)
            yolo_msg = None
            
        except CvBridgeError as e:
            print(e)
  
    def detectID(self, yolo_msg, yolo_img):
        yolo_img = cv2.cvtColor(yolo_img, cv2.COLOR_BGR2GRAY)
        for i in range(self.numMarkers):
            if yolo_msg == self.numMarkers.name:
                markerID == self.numMarkers.ID
            ##Exportar 
            #path = "/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/detector/markers_alma/"
            #sift_export = cv2.imwrite(path+'detected_'+str(time.strftime("%Y%m%d-%H%M%S"))+'.bmp', marker_detected_by_homography) 
            print("the marker found ID is", markerID)      
        return markerID
             
    def sortCorners(self, img, corners, w_center, h_center):
        sorted_corner = [list(), list(), list(), list()]
        #print (corners)
        for corner in corners:
            if corner[0][0] <= w_center and corner[0][1] <= h_center: # primer cuadrante
                sorted_corner[0] = corner[0]
                cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.corner_colors[0], -1)
                #print("First Cuadrant")
            elif corner[0][0] > w_center and corner[0][1] <= h_center: # segundo cuadrante
                sorted_corner[1] = corner[0]
                cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.corner_colors[1], -1)
                #print("Second Cuadrant")
            elif corner[0][0] > w_center and corner[0][1] > h_center: # tercero cuadrante
                sorted_corner[2] = corner[0]
                cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.corner_colors[2], -1)
                #print("Third Cuadrant")
            elif corner[0][0] <= w_center and corner[0][1] > h_center: # tercero cuadrante
                sorted_corner[3] = corner[0]
                cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.corner_colors[3], -1)
                #print("Fourth Cuadrant")
            else:
                print("Error of Cuadrant")
                return 0
        else:
            #print("Sorted corners")
            #print(sorted_corner)
            return sorted_corner

    def makeMsgMarker(self, numMarkers, corners):
        new_marker = msg_marker()
        #new_marker = [corners, 0, 0, numMarkers]
        for corner in corners:
            pixel = Point32()
            pixel.x = corner[0]
            pixel.y = corner[1]
            pixel.z = 0
            #print(pixel)
            new_marker.Corners.append(pixel)
        else:
            new_marker.map = UInt8(0)
            new_marker.sector = UInt8(0)
            new_marker.ID = UInt8(numMarkers)
            #print(new_marker)
            return new_marker
        print("Error make marker msg")

def main(args):
    rospy.init_node('detector_SIFT', anonymous=True)
    detector(5)  #num imgs de marcas a buscar
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)