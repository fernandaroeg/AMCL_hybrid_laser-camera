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

class detector:
    def __init__(self, numImages):
        self.bridge = CvBridge()
        self.pub_marker = rospy.Publisher('detected_markers', messagedet, queue_size=1)
        self.pub_projection = rospy.Publisher('detector_img', Image, queue_size=1)
        self.pub_num_marker = rospy.Publisher('num_detecte_markers', num_markers, queue_size=10)
        self.pub_num_points_cloud = rospy.Publisher('num_points_cloud', num_points_cloud, queue_size=20)
        self.colours = ((255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255)) ## BGR coners

        #Initialize SIFT detector and feature matcher 
        self.numMarker = numImages
        numFeatures = 200 # max image features numbers
        self.thresh = 70 # umbral para pasar la imagen a escala de grises
        self.min_w_h_image = 10 # minimun weidth and height image
        self.thresh_biary = cv2.THRESH_BINARY_INV # cv2.THRESH_BINARY_INV busca objetos oscuros con fondo claro y cv2.THRESH_BINARY_ busca objetos claros con fondo oscuro
        self.numMatches = 15
        self.minNumMatches = 15
        self.maxDist = 60
        self.minDist = 5 # minimun distance between the first and second distance        
        
        #SIFT
        #self.detector = cv2.ORB_create(numFeatures) #Iniciar detector ORB is a fusion of FAST keypoint detector and BRIEF descriptor
        self.detector = cv2.xfeatures2d.SIFT_create()
        self.bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True) # crea BFMatcher object feature matcher, con normalizacion 1/2 y el crosscheck pending...
        
        marker= "marker1"
        path = str("/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/detector/markers_alma/"+marker+".png")  #PENDING modificar para tomar path desde launch file                                          
        print(path)
        
        self.img  = cv2.imread(path)
        self.img  = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

        #cv2.imshow('original', self.img)
        #cv2.waitKey(0)   
        self.image_sub = rospy.Subscriber("/camera/RGB1/Image", Image, self.callback) #PENDING de aqui toma la info. 
        

    def callback(self, data):  #READ image from RGB/image_raw topic and convert to cv2 object
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.original_width  = cv_image.shape[1]
            self.original_height = cv_image.shape[0]
            
            #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            #kp, des = self.detector.detectAndCompute(self.img, None) #find keypoints and descriptors
            #kp2, des2 = self.detector.detectAndCompute(cv_image, None)
            #  
            #matches = self.bf.match(des, des2)
            #matches = sorted(matches, key = lambda x:x.distance) #Find matches with 2nd images
            #self.img3 = cv2.drawMatches(self.img, kp, cv_image, kp2, matches[:600], cv_image, flags=2)
            
            ###print ("publicando img del topic en cv window:")
            ###image_detected = self.bridge.cv2_to_imgmsg(self.img3, encoding="bgr8")
            ###self.pub_projection.publish(image_detected)
            self.findContours(cv_image)

        except CvBridgeError as e:
            print(e)
        #self.findContours(cv_image)
        #cv2.waitKey(3)

    def findContours(self, img):
        print ("!!!!!ENTRAMOS a findContours 1/5")
        blurred_img = cv2.GaussianBlur(img, (5,5), 0)  #add gaussian filter, to improve border detection PENDING
        hsv = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2HSV) #convert to hsv color space
        h,s,v = cv2.split(hsv) #split the channels
        
        th, threshed = cv2.threshold(v, self.thresh, 255, self.thresh_biary) #define threshold
        cnts = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]## cnts is a list with len iqual to contours number, find contours using threshold

        canvas  = img.copy()
        canvas = cv2.drawContours(canvas, cnts, -1, (0,255,0), 1)
        ####print ("publicando img de find contours:")
        ####visualizar = np.concatenate((img, canvas), axis=1)
        ####image_detected = self.bridge.cv2_to_imgmsg(visualizar, encoding="bgr8")
        ####self.pub_projection.publish(image_detected) 
        self.findSquares(cnts, canvas, img)

    def findSquares(self, cnts, canvas, original_img):
        print ("!!!!!ENTRAMOS a findsquares 2/5")
        num_cuadro = 0 #contador para poner nombre a las imgs de cuadros recortados
        for cnt in cnts: # Un bucle por cada contorno      
            num_cuadro = num_cuadro + 1 #contador para poner nombre a las imgs de cuadros recortados
            arclen = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02* arclen, True)  ###IMPORTANTE! esta es la funcion que encuentra los rectangulos  
            if len(approx) == 4: ## solo entramos si es un rectangulo, four corners
                x1 ,y1, w, h = cv2.boundingRect(approx)
                area=w*h  
                aspectRatio = float(w)/h
                cuadros = cv2.drawContours(original_img, [approx], -1, (255, 0, 0), 1, cv2.LINE_AA)
                #hacemos calculos para recortar
                w_img_ = list()
                h_img_ = list()
                count2 = 0
                for point in approx:
                    w_img_.append(point[0][0])
                    h_img_.append(point[0][1])
                    orillas = canvas
                    #cuadros = cv2.circle(original_img, (point[0][0],point[0][1]), 2, self.colours[count2], -1)
                    ###visualizar = np.concatenate((orillas, cuadros), axis=1)
                    ###image_detected = self.bridge.cv2_to_imgmsg(visualizar, encoding="bgr8")
                    ###self.pub_projection.publish(image_detected)
                    count2+=1
                w_img = sorted(w_img_)
                h_img = sorted(h_img_)
                w_img_max = w_img[0]+w
                h_img_max = h_img[0]+h
                
                #lookup transform funcion de tf que multiplica matrices de transformacion
                if w >= self.min_w_h_image  and  h >= self.min_w_h_image: #and aspectRatio >= 2.5 and area>100: #la marca debe ser cuadrada y al menos de 10x10pixels
                    print("FOUND GOOD SQUARE in red!!!!!")
                    cv2.drawContours(cuadros, [approx], -1, (0, 0, 255), 1, cv2.LINE_AA)
                    crop_img = original_img[h_img[0]:h_img_max, w_img[0]:w_img_max] ## !!!!!!Recortamos la imagen del contorno                    
                    path = "/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/detector/markers_alma/"
                    square_img = cv2.imwrite(path+'square_det_'+str(num_cuadro)+'.bmp', crop_img)
                #cv2.rectangle(cuadros, (5,5),(self.original_width-5,self.original_height-5),(0,0,255),1) #cuadro de Miguel para dejar orilla de 5pixeles
                cv2.rectangle(cuadros, (5,5),(15,15),(255,0,0),1) #medida minima para buscar marcas, cuadro de 10x10pixels
                visualizar = np.concatenate((orillas, cuadros), axis=1)
                image_detected = self.bridge.cv2_to_imgmsg(visualizar, encoding="bgr8")
                self.pub_projection.publish(image_detected)
                #w_center = w_img[0] + int(width/2)
                #h_center = h_img[0] + int(height/2)
                    
                    #!!!Aplicamos el clasificador SILF
                    #######new_msg_marker = messagedet()
                    #######marker = self.compareImage(crop_img)
                    #######if  marker >= 0: # >=0 si ha encontrado una marca
                    #######    print("marca encontrada: " + str(marker))
                    #######    cv2.drawContours(canvas, [approx], -1, (0, 0, 255), 4, cv2.LINE_AA)
                    #######    text = str("Id:" + str(marker))
                    #######    cv2.putText(canvas, text, (w_center, h_center), 2, 1, (255, 0, 0), 2, lineType=cv2.LINE_AA)
                    #######    corners = self.sortCorners(canvas, approx, w_center, h_center)
                    #######    #print(corners)
                    #######    if len(corners) == 4:
                    #######        new_msg_marker.DetectedMarkers.append(self.makeMsgMarker(marker, corners)) 
                #else:
                #    print("not valid")
        
        #####IMPORTANTE LOGICA PARA PUBLICAR MSG DE marca detectada
        #####if len(new_msg_marker.DetectedMarkers)>0:
        #####    #print("publico -> " + str(len(new_msg_marker.DetectedMarkers)))
        #####    new_msg_marker.header.seq = rospy.Duration()
        #####    new_msg_marker.header.stamp = rospy.Time.now()
        #####    new_msg_marker.header.frame_id = "camera_link"
        #####    #print(new_msg_marker)
        #####    self.pub_marker.publish(new_msg_marker)
            
        ##############MOSTRAR LA IMAGEN DETECTADA Y PUBLICAR EN TOPIC
        #cv2.imshow(self.img_canvas, canvas)
        #print(self.img_canvas)
        #####cv2.imshow(self.img_canvas, cv2.resize(canvas,(950,540)))
        #####image_message = self.bridge.cv2_to_imgmsg(canvas, encoding="passthrough")
        #####self.pub_projection.publish(image_message)
        #####n_markers = num_markers()
        #####n_markers.header.stamp = rospy.Time.now()
        #####n_markers.number = UInt8(len(new_msg_marker.DetectedMarkers))
        #####self.pub_num_marker.publish(n_markers)

    def compareImage(self, img):
        print ("!!!!!ENTRAMOS a compareImage, el + impo. aqui se hace el SIFT 3/5")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kp, des = self.detector.detectAndCompute(img, None)
        if len(kp) > 0  and  len(des) > 20:
            print("###################################")
            print("img[" + str(img.shape[0]) + ", " + str(img.shape[1]) + "]")

            # Match descriptors.
            pos = list()
            matches = list()
            dist = list()
            count = 0
            for i in range (0, self.numMarker):
                marches_ = self.bf.match(self.markers[2][i], des)
                if len(marches_) >= self.minNumMatches: # si no hay minimo 10 coincidencias se descarta
                    matches.append(marches_)
                    matches[count] = sorted(matches[count], key = lambda x:x.distance)
                    pos.append(i)
                    dist.append(list())             
                    for m in matches[count]:
                        dist[count].append(m.distance)
                    count += 1
                #else:
                    #print("no valid matches " + str(i))

            if len(pos) > 0: # al menos tiene que haber una
                # Coincidencias.
                poseDist = list()
                for i in range (0, len(pos)):
                    poseDist.append(int(sum(dist[i][:(self.numMatches - 1)])/self.numMatches)) 
                    cncd = int (len(matches[i]) * 100) / int (len(self.markers[1][i]))
                    print("des" + str(pos[i]) + ": " + str(len(self.markers[1][i])) + "  ,des: " + str(len(kp)) + "  ,matches: " + str(len(matches[i])) + " -> " + str(cncd) + ", d(Total): " + str(int(sum(dist[i])/len(dist[i]))) + " , d(:" + str(self.numMatches) + ")-> " + str(poseDist[i]) + " , d(:19)-> " + str(int(sum(dist[i][:19])/20)) + " , d(:39)-> " + str(int(sum(dist[i][:39])/40)))
                
                poseDistShorted = sorted(poseDist)
                if (poseDistShorted[1] - poseDistShorted[0]) < self.minDist: # distancia minima entre el primero y el segundo con menos distancia
                    print("no min distance" )
                    return -1

                for j in range (0, len(pos)):
                    if poseDist[j] == poseDistShorted[0] and poseDistShorted[0] <= self.maxDist:
                        #print("***** It is marker: " + str(pos[j]))
                        img3 = cv2.drawMatches(self.markers[0][pos[j]], self.markers[1][pos[j]], img, kp, matches[j][:self.numMatches], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                        #cv2.imshow("Classifed Marker", img3) esta muestra la clasificacion
                        return pos[j]
            else:
                print("there are not valid matches")
                return -1
                #img3 = cv2.drawMatches(self.marker0,self.kp0,img,kp,matches[:60],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                #plt.imshow(img3),plt.show()
        else:
            return -1
            

    def sortCorners(self, img, corners, w_center, h_center):
        print ("!!!!!casi listos, acomodamos las esquinas detectadas para el mensaje 4/5")
        sorted_corner = [list(), list(), list(), list()]
        #print (corners)
        for corner in corners:
            if corner[0][0] <= w_center and corner[0][1] <= h_center: # primer cuadrante
                sorted_corner[0] = corner[0]
                cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.colours[0], -1)
                #print("First Cuadrant")
            elif corner[0][0] > w_center and corner[0][1] <= h_center: # segundo cuadrante
                sorted_corner[1] = corner[0]
                cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.colours[1], -1)
                #print("Second Cuadrant")
            elif corner[0][0] > w_center and corner[0][1] > h_center: # tercero cuadrante
                sorted_corner[2] = corner[0]
                cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.colours[2], -1)
                #print("Third Cuadrant")
            elif corner[0][0] <= w_center and corner[0][1] > h_center: # tercero cuadrante
                sorted_corner[3] = corner[0]
                cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.colours[3], -1)
                #print("Fourth Cuadrant")
            else:
                print("Error of Cuadrant")
                return 0
        else:
            #print("Sorted corners")
            #print(sorted_corner)
            return sorted_corner

    def makeMsgMarker(self, numMarker, corners):
        print ("!!!!!CONGRATS, se manda el mensaje makeMsgMarker 5/5")
        #print("Make msg marker")
        #print(corners)
        new_marker = msg_marker()
        #new_marker = [corners, 0, 0, numMarker]
        for corner in corners:
            pixel = Point32()
            #print(corner)
            pixel.x = corner[0]
            pixel.y = corner[1]
            pixel.z = 0
            #print(pixel)
            new_marker.Corners.append(pixel)
        else:
            new_marker.map = UInt8(0)
            new_marker.sector = UInt8(0)
            new_marker.ID = UInt8(numMarker)
            #print(new_marker)
            return new_marker
        print("Error make marker msg")

def main(args):
    rospy.init_node('detector_SILF', anonymous=True)
    detector(4)  #num self.imgs con las marcas
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)