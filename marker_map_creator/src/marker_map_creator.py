#1 usando la estructura de miguel donde se hace un pre procesamiento y luego se inicia un nodo
#   en el pre-procesamiento
#   Leer del LAUNCH FILE
#       Pa calcular los puntos: input en launch file imagen depth, array 4 puntos rectangulo, bag file groundtruth
#       Pa calcular la posicion en el espacio: timestamp de la imagen, tf de la camara que lo tomo
#   cv read img depth, procesar en la imagen los 4 puntos (for 4 veces) y ponerles las ec del pc
#   calcular su posicion en el espacio:
#       1. abrir bag file del groundtruth y en un ciclo for encontrar el tstamp más cercano al de la img
#       2.      if tstamp_img - tstamp[i] > resta anterior  && >0
#                           pos_img = x[i], y[i], z[i], yaw[i]
#       3. ya que tengo la posicion del robot en ese instante calcular la transformada de mis puntos 
#       4. publicar los puntos como marcas en rviz
#       5. exportar  los datos en archivo yaml con formato necesario
#       6. puedo hacer un loop de todo esto?
#
#
#
#
#
#
#
#
#