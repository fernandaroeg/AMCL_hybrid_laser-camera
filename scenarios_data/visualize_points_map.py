import matplotlib.pyplot as plt
map_file = '/home/fer/Desktop/catkin_ws/src/ROS_AMCL_Hybrid_Localization/maps/alma_fullhouse_pointsmap.txt'
mapX = []
mapY = []

ground_truth = '/home/fer/Desktop/catkin_ws/src/ROS_AMCL_Hybrid_Localization/maps/alma_log_estimated_path.txt'
poseX = []
poseY = []
theta = []

odom_path = '/home/fer/.ros/odom_debug.txt'
#odom_path = '/home/fer/Desktop/catkin_ws/src/ROS_AMCL_Hybrid_Localization/maps/alma_odom_debug.txt'
odomX = []
odomY = []

with open(map_file,'r') as file:
    file_data = file.readlines()
    for line in file_data:
        breaking_lines = line.split()
        mapX.append(float(breaking_lines[0]))
        mapY.append(float(breaking_lines[1]))
		
with open(ground_truth,'r') as file:
    file_data = file.readlines()
    for line in file_data:
        breaking_lines = line.split()
        poseX.append(float(breaking_lines[1]))
        poseY.append(float(breaking_lines[2]))
        theta.append(float(breaking_lines[3]))

with open(odom_path,'r') as file:
    file_data = file.readlines()
    for line in file_data:
        breaking_lines = line.split()
        odomX.append(float(breaking_lines[0]))
        odomY.append(float(breaking_lines[1]))
		
# MAP Scatterplot
plt.scatter(mapX,   mapY,   0.1, color='blue')
plt.scatter(poseX,  poseY,  0.5, color='r')
plt.scatter(odomX, odomY, 0.5, color='g')
plt.plot(poseX[0],poseY[0], 'go') 
plt.plot(poseX[-1],poseY[-1], 'ro') 

plt.title('Map by points and groundtruth')
plt.xlabel('X - value')
plt.ylabel('Y - value')
plt.show()