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
        #self.thresh        = 30 # umbral para pasar la imagen a escala de grises #PENDG al agregar otsi thresh este param ya no es necesario
        #Rectangle detection parameters
        self.min_w_h_image = 10 # minimun weidth and height image
        #SIFT parameters
        numFeatures        = 100 # max image features numbers
        self.numMatches    = 50
        self.minNumMatches = 15
        self.maxDist        = 60
        self.minDist        = 5 # minimun distance between the first and second distance   
        
        #INICIALIZAR SIFT
        #self.detector = cv2.ORB_create(numFeatures) #Iniciar detector ORB is a fusion of FAST keypoint detector and BRIEF descriptor
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
            self.original_width  = cv_image.shape[1]
            self.original_height = cv_image.shape[0]
            
            #1. Find contours in the image received from topic
            contours, contours_img, threshold_img = self.findContours(cv_image)
            
            #2. Find rectangles using the contours
            found_rectangle, rectangles_img, rectangle_cropped, rectangle_crop_points = self.findRectangles(contours, contours_img, cv_image)
            
            #3. Compare found rectangle to desired markers with SIFT algorithm
            if found_rectangle == True:
                marker = self.compareImage(rectangle_cropped)  
                #4. If marker was found sort corners
                if  marker >= 0: 
                    corners = self.sortCorners(canvas, approx, w_center, h_center)
                    #5. Publish Marker Msg in TOPIC
                    if len(corners) == 4:
                        msg_marker.DetectedMarkers.append(self.makeMsgMarker(marker, corners))   
                        msg_marker.header.seq = rospy.Duration()
                        msg_marker.header.stamp = rospy.Time.now()
                        msg_marker.header.frame_id = "camera_link"
                        self.pub_marker.publish(msg_marker)
                        n_markers = num_markers()
                        n_markers.header.stamp = rospy.Time.now()
                        n_markers.number = UInt8(len(msg_marker.DetectedMarkers))
                        self.pub_num_marker.publish(n_markers)
                        print("publico -> " + str(len(msg_marker.DetectedMarkers)))
                        print(new_msg_marker)
                    
            ##6. MOSTRAR LA IMAGEN DETECTADA Y PUBLICAR EN TOPIC
            #cv2.rectangle(cuadros, (5,5),(self.original_width-5,self.original_height-5),(0,0,255),1) #cuadro de Miguel para dejar orilla de 5pixeles
            #cv2.rectangle(cuadros, (5,5),(15,15),(255,0,0),1) #medida minima para buscar marcas, cuadro de 10x10pixels
            contours_concat = np.concatenate((threshold_img, contours_img), axis=1)
            visualizar   = np.concatenate((contours_concat, rectangles_img), axis=1)
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
        N = len(points) # Stores numMarkersMatched of edges in polygon
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
        blurred_img = cv2.GaussianBlur(img, (5,5), 0)  #PENDING PRUEBAS FER add gaussian filter, to improve border detection PENDING
        hsv = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV) #convert to hsv color space
        h,s,v = cv2.split(hsv) #split the channels
        
        #th, threshed = cv2.threshold(v, self.thresh, 255, self.thresh_binary) #define threshold
        th, threshed = cv2.threshold(v, 0, 255, self.thresh_binary | cv2.THRESH_OTSU)     
        
        v_bgr = cv2.cvtColor(v, cv2.COLOR_GRAY2BGR) #convert thresh img to 3 channels to concatenate
        v_bgr = np.concatenate((hsv, v_bgr), axis=1)
        threshed_bgr = cv2.cvtColor(threshed, cv2.COLOR_GRAY2BGR) #convert thresh img to 3 channels to concatenate
        threshold_img = np.concatenate((v_bgr, threshed_bgr), axis=1)

        contours = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]## contours is a list with len iqual to contours number, find contours using threshold

        contours_img  = img.copy()
        contours_img = cv2.drawContours(contours_img, contours, -1, (0,255,0), 1)

        return contours, contours_img, threshold_img

    def findRectangles(self, contours, contours_img, original_img):
        timedate = time.strftime("%Y%m%d-%H%M%S") 
        new_msg_marker = messagedet()
        found_rectangle = False  
        rectangle_crop_img = np.zeros((100,100,3), dtype=np.uint8) #return empty img
        rectangles_img = original_img
        rectangle_points = 0
        
        for cnt in contours: # Un bucle por cada contorno      
            arclen = cv2.arcLength(cnt, True)
            rectangle_points = cv2.approxPolyDP(cnt, 0.02* arclen, True)  ###IMPORTANTE! esta es la funcion que encuentra los rectangulos  
           
            if len(rectangle_points) == 4: ## solo entramos si es un rectangulo, four corners
                x1 ,y1, w, h = cv2.boundingRect(rectangle_points)
                area=w*h  
                aspectRatio = float(w)/h
                aspectRatio2 = float(h)/w
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
                if w >= self.min_w_h_image  and  h >= self.min_w_h_image and self.isConvex(convex_list): #and aspectRatio <= 1.5 and aspectRatio2 <= 2 and area>250:
                    #print("FOUND GOOD SQUARE in red!!!!!")
                    rectangles_img =cv2.drawContours(rectangles_img, [rectangle_points], -1, (0, 0, 255), 1, cv2.LINE_AA)
                    rectangle_crop_img = original_img[h_img[0]:h_img_max, w_img[0]:w_img_max] ## !!!!!!Recortamos la imagen del contorno   
                    found_rectangle = True
                    path = "/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/detector/markers_alma/"
                    #rectangle_export = cv2.imwrite(path+'square_det_'+str(timedate)+'.bmp', rectangle_crop_img) #PENDING NOT EXPORT
                    
        return found_rectangle, rectangles_img, rectangle_crop_img, rectangle_points

    def compareImage(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kp, des = self.detector.detectAndCompute(img, None)
        if len(kp) > 0  and  len(des) > 20:
            print("#####Croped rectangle to be compared#####"+"img[" + str(img.shape[0]) + ", " + str(img.shape[1]) + "]")
            print("num kps:" + str(len(kp)) + "num des:" + str(len(des)))
            minFeat_matches_found = list()
            markerFoundID = list()
            dist = list()
            numMarkersMatched = 0
            for i in range (0, self.numMarkers): #vamos a comparar la img rect detectada con todos los markers deseados
                matches_found = self.bf.match(self.markers[2][i], des)
                print("Comparing croped rect to marker", i, "Num of matches is", len(matches_found))
                
                if len(matches_found) >= self.minnumMarkersMatchedes: 
                    minFeat_matches_found.append(matches_found) #almacenamos en 1 lista los matches que cumplen el min requerido de features
                    
                    minFeat_matches_found[numMarkersMatched] = sorted(minFeat_matches_found[numMarkersMatched], key = lambda x:x.distance) #extract distance from matched points https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html
                    markerFoundID.append(i) #num marker con el que se hizo match
                    dist.append(list())             
                    for m in minFeat_matches_found[numMarkersMatched]:
                        dist[numMarkersMatched].append(m.distance) #meter distancias a una lista 
                    numMarkersMatched += 1  #numMarkersMatched es el numero de markers para los que se encontro coincidencia c/el rectangulo en cuestion 
                    
            # Coincidencias.
            if numMarkersMatched>0: #si se encontro al menos 1 coincidencia entonces...
                poseDist = list()
                for i in markerFoundID: #analizamos cada uno de los markers
                    poseDist.append(int(sum(dist[i][:(self.numMarkersMatched - 1)])/self.numMarkersMatched)) #promedio de distancia, suma de la distancia de los encontrados entre el total de encontrados
                
                poseDistShorted = sorted(9)
                if (poseDistShorted[1] - poseDistShorted[0]) < self.minDist: # distancia minima entre el primero y el segundo con menos distancia
                    print("no min distance" )
                    return -1
                
                for j in range (0, len(markerFoundID)):
                    if poseDist[j] == poseDistShorted[0] and poseDistShorted[0] <= self.maxDist:
                        print("***** It is marker: " + str(markerFoundID[j]))
                        img3 = cv2.drawMatches(self.markers[0][markerFoundID[j]], self.markers[1][markerFoundID[j]], img, kp, matches[j][:self.numMarkersMatchedes], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                        cv2.imshow("Classifed Marker", img3) #esta muestra la clasificacion
                        return markerFoundID[j]     
                #cv2.drawContours(canvas, [approx], -1, (0, 0, 255), 4, cv2.LINE_AA)
                #text = str("Id:" + str(marker))
                #cv2.putText(canvas, text, (w_center, h_center), 2, 1, (255, 0, 0), 2, lineType=cv2.LINE_AA)
                crop_sift_to_marker = cv2.drawMatches(self.markers[0][i], self.markers[1][i], img, kp, matches_found[:600], img, flags=2)
                path = "/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/detector/markers_alma/"
                sift_export = cv2.imwrite(path+'sift_'+str(time.strftime("%Y%m%d-%H%M%S"))+'.bmp', crop_sift_to_marker)  
            else:
                print("no valid matches for marker:" + str(i))
                return -1
        else:
            return -1
             
    def sortCorners(self, img, corners, w_center, h_center):
        print ("!!!!!casi listos, acomodamos las esquinas detectadas para el mensaje 4/5")
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
        #print("Make msg marker")
        #print(corners)
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