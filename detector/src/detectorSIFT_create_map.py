#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from detector.msg import messagedet, num_markers
from detector.msg import marker as msg_marker
from geometry_msgs.msg import Point32
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image
import os
import time


class detector:
    def __init__(self):
        #ROS configuration
        self.bridge = CvBridge()
        self.pub_marker     = rospy.Publisher('detected_markers', messagedet, queue_size=1)
        self.pub_projection = rospy.Publisher('detector_img', Image, queue_size=1)
        self.pub_num_marker = rospy.Publisher('num_detecte_markers', num_markers, queue_size=10)
       
        #Parameters SETUP
        self.numMarkers     = rospy.get_param('/detectorSIFT/num_markers')
        self.corner_colors  = ((255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255)) #BGR coners
        self.export_img_det = True
        self.export_path    = "/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/detector/markers_alma/"
        
        #Contour detection parameters, THRESH_BINARY_INV busca objetos oscuros con fondo claro, THRESH_BINARY busca objetos claros con fondo oscuro
        self.thresh_binary  = cv2.THRESH_BINARY_INV 
        #Rectangle detection parameters minimun width and height for images to loook
        self.min_w_h_image = 10 
        #Min number of features matched between analyzed rectangle and stored marker to be consider as a detection
        self.minNumMatches = 10
        
        #INICIALIZAR SIFT
        self.SIFTdetector = cv2.xfeatures2d.SIFT_create()
        
        #CREAR LISTA DE MARCADORES A BUSCAR
        self.markers = (list(), list(), list()) #se crea lista3D para almacenar img, keypoints, descriptors de cada uno de los marcadores 
        for i in range(0, self.numMarkers):
            markers_path = rospy.get_param('/detectorSIFT/markers_path')
            path = str(markers_path + "marker" + str(i) +".png") 
            print(path)
            img = cv2.imread(path, 0)
            self.markers[0].append(img)
            kp, des = self.SIFTdetector.detectAndCompute(img, None)
            self.markers[1].append(kp)
            self.markers[2].append(des)   
        print("total images in folder: " + str(len(self.markers[0])))
        
        #START detector process when img is received in topic
        self.image_sub = rospy.Subscriber("/camera/RGB1/Image", Image, self.callback)
        

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            #1. Find contours in the image received from topic
            contours, contours_img, threshold_img = self.findContours(cv_image)
            
            #2. Find rectangles using the contours
            found_rectangle, rectangles_img, rectangle_cropped, rectangle_crop_points, rect_w_center, rect_h_center = self.findRectangles(contours, contours_img, cv_image)
            
            #3. Compare found rectangle to markers stored in self.markers
            if found_rectangle == True:
                markerID = self.compareImage(rectangle_cropped)
                
                #4. If marker was found sort corners
                if  markerID >= 0: 
                    corners = self.sortCorners(cv_image, rectangle_crop_points, rect_w_center, rect_h_center)
                    #5. Publish Marker Msg in TOPIC
                    if len(corners) == 4:
                        msg_marker = messagedet() 
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
            #cv2.rectangle(cuadros, (5,5),(self.original_width-5,self.original_height-5),(0,0,255),1) #cuadro de Miguel para dejar orilla de 5pixeles
            #cv2.rectangle(cuadros, (5,5),(15,15),(255,0,0),1) #medida minima para buscar marcas, cuadro de 10x10pixels
            contours_concat = np.concatenate((threshold_img, contours_img), axis=1) 
            visualizar = np.concatenate((contours_concat, rectangles_img), axis=1)
            detector_img = self.bridge.cv2_to_imgmsg(visualizar, encoding="bgr8")
            self.pub_projection.publish(detector_img)
            
        except CvBridgeError as e:
            print(e)
        
    def CrossProduct(self, A):
        X1 = (A[1][0] - A[0][0]) # Stores coefficient of X # direction of vector A[1]A[0]
        Y1 = (A[1][1] - A[0][1]) # Stores coefficient of Y # direction of vector A[1]A[0]
        X2 = (A[2][0] - A[0][0]) # Stores coefficient of X # direction of vector A[2]A[0]
        Y2 = (A[2][1] - A[0][1]) # Stores coefficient of Y # direction of vector A[2]A[0] 
        return (X1 * Y2 - Y1 * X2) # Return cross product
 
    def isConvex(self, points): # Function to check if the polygon is convex or not https://www.geeksforgeeks.org/check-if-given-polygon-is-a-convex-polygon-or-not/
        N = len(points) # Stores i of edges in polygon
        prev = 0 # Stores direction of cross product of previous traversed edges
        curr = 0 # Stores direction of cross product of current traversed edges
        for i in range(N):     # Traverse the array
            temp = [points[i], points[(i + 1) % N],
                            points[(i + 2) % N]] # Stores three adjacent edges of the polygon
            curr = self.CrossProduct(temp)  # Update curr
            if (curr != 0):  # If curr is not equal to 0
                if (curr * prev < 0):  # If direction of cross product of all adjacent edges are not same
                    return False
                else:
                    prev = curr   # Update curr
        return True

    def findContours(self, img):
        #1. Change color space and extract gray channel
        blurred_img = cv2.GaussianBlur(img, (5,5), 0)  
        hsv = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV) #convert to hsv color space
        h,s,v = cv2.split(hsv) #split the channels
        
        #2. Apply umbralization 
        th, threshed = cv2.threshold(v, 0, 255, self.thresh_binary | cv2.THRESH_OTSU) #Use Otsu automatic thresholding value 
        
        #3. Find Contours in the umbralized image
        contours = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]## contours is a list with len = contours number, find contours using threshold
        
        #Export images of color space change, grayscale and thresholding 
        v_bgr = cv2.cvtColor(v, cv2.COLOR_GRAY2BGR) #convert thresh img to 3 channels to concatenate
        v_bgr = np.concatenate((hsv, v_bgr), axis=1)
        threshed_bgr = cv2.cvtColor(threshed, cv2.COLOR_GRAY2BGR) #convert thresh img to 3 channels to concatenate
        threshold_img = np.concatenate((v_bgr, threshed_bgr), axis=1)
        #Export image of contour detection
        contours_img  = img.copy()
        contours_img = cv2.drawContours(contours_img, contours, -1, (0,255,0), 1)

        return contours, contours_img, threshold_img

    def findRectangles(self, contours, contours_img, original_img):
        timedate = time.strftime("%Y%m%d-%H%M%S") 
        found_rectangle = False  
        rectangle_crop_img = np.zeros((100,100,3), dtype=np.uint8) #return empty img
        rectangles_img = original_img
        rectangle_points = 0
        
        for cnt in contours: # Un bucle por cada contorno      
            arclen = cv2.arcLength(cnt, True)
            rectangle_points = cv2.approxPolyDP(cnt, 0.02* arclen, True)  ###IMPORTANTE! esta es la funcion que encuentra los rectangulos/poligonos
           
            if len(rectangle_points) == 4: ## solo entramos si es un rectangulo, four corners
                x1 ,y1, w, h = cv2.boundingRect(rectangle_points)
                rectangles_img = cv2.drawContours(original_img, [rectangle_points], -1, (255, 0, 0), 1, cv2.LINE_AA)
                #hacemos calculos para recortar
                w_img_ = list()
                h_img_ = list()
                for point in rectangle_points:
                    w_img_.append(point[0][0])
                    h_img_.append(point[0][1])
                    canvas = contours_img
                    orillas = canvas
                w_img = sorted(w_img_)
                h_img = sorted(h_img_)
                w_img_max = w_img[0]+w
                h_img_max = h_img[0]+h
                w_center = w_img[0] + int(w/2)
                h_center = h_img[0] + int(h/2)
                #Las sig. 4 lineas son para adaptar el formato de la lista de puntos a la funcion isconvex
                approx_list = np.ndarray.tolist(rectangle_points)
                convex_list = []
                for i in range(0,4):
                    convex_list.append(approx_list[i][0])
                    
                #CONDICIONES PARA FILTRAR CUADROS tam minimo y poligono convexo
                if w >= self.min_w_h_image  and  h >= self.min_w_h_image and self.isConvex(convex_list):
                    rectangles_img =cv2.drawContours(rectangles_img, [rectangle_points], -1, (0, 0, 255), 1, cv2.LINE_AA)
                    rectangle_crop_img = original_img[h_img[0]:h_img_max, w_img[0]:w_img_max] #Recortamos la imagen del contorno   
                    found_rectangle = True
                    #path = "/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/detector/markers_alma/"
                    #rectangle_export = cv2.imwrite(path+'square_det_'+str(timedate)+'.bmp', rectangle_crop_img)
            else:
                w_center = 0
                h_center = 0
                    
        return found_rectangle, rectangles_img, rectangle_crop_img, rectangle_points, w_center, h_center

    def compareImage(self, rectangle):
        rectangle_original = rectangle
        rectangle = cv2.cvtColor(rectangle, cv2.COLOR_BGR2GRAY)
        kp, des = self.SIFTdetector.detectAndCompute(rectangle, None)
        
        if len(kp) > 0 and len(des)>25: #There must be at least 1 feature to compare and des>20 helps avoiding this error https://stackoverflow.com/questions/25089393/opencv-flannbasedmatcher
            minNumMatches_found = list()
            for i in range(self.numMarkers): #compare rectangle to all the markers stored, logic from:https://docs.opencv.org/3.4/d1/de0/tutorial_py_feature_homography.html         
                #1. Compute flann matches between rectangle and markers
                FLANN_INDEX_KDTREE = 1 
                index_params  = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
                search_params = dict(checks = 50)
                flann = cv2.FlannBasedMatcher(index_params, search_params)
                matches_found = flann.knnMatch(self.markers[2][i],des,k=2) #updated matcher from bf to flann in order to perform lowe's ratio test
                print "Comparing rect to marker", i
                #print "Num unfiltered matches found ", len(matches_found)
                
                #2. Store good matches that pass the Lowe's ratio test.
                good_matches = []
                for m,n in matches_found:
                    if m.distance < 0.7*n.distance:
                        good_matches.append(m)
                        markerID = i
                #print("Filtered num of matches per Lowe's tests is", len(good_matches))
                
                #3. If the number of good matches found is >= minNumMatches, then use homography matrix and RANSAC algorithm to detect the marker in the rectangle image
                MIN_MATCH_COUNT = self.minNumMatches
                if len(good_matches)>MIN_MATCH_COUNT:
                    src_pts = np.float32([ self.markers[1][i][m.queryIdx].pt for m in good_matches ]).reshape(-1,1,2)
                    dst_pts = np.float32([ kp[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)
                
                    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                    matchesMask = mask.ravel().tolist()
                
                    h,w = rectangle.shape
                    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                    dst = cv2.perspectiveTransform(pts,M)
                
                    #Draw detected marker in the original image 
                    #marker_detected_by_homography = cv2.polylines(self.markers[0][markerID],[np.int32(dst)],True,(0,255,0),3, cv2.LINE_AA)
                    #PENDING add id leyend
                    #marker_detected_by_homography = cv2.polylines(original_img,[np.int32(dst)],True,(0,255,0),3, cv2.LINE_AA)
                    marker_detected_by_homography = cv2.polylines(rectangle_original,[np.int32(dst)],True,(0,255,0),3, cv2.LINE_AA)                    
                                        
                    #Exportar image to folder
                    if self.export_img_det == True:
                        sift_export = cv2.imwrite(self.export_path+'detected_'+str(time.strftime("%Y%m%d-%H%M%S"))+'.bmp', marker_detected_by_homography) 
                    
                    #Function Output
                    print "The marker found ID is", markerID
                    return markerID
                else:
                    print "      Not enough matches are found - %d/%d" % (len(good_matches),MIN_MATCH_COUNT)
                 
        else:
            return -1
             
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
    detector()  
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)