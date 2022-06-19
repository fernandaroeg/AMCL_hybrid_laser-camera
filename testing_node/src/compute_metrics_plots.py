#! /usr/bin/env python
#Ubuntu 16.04, ROS Kinetic, python 2.7
#Script that process text files with pose data to compute metrics and export plots. 
import rospy
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

path= "/home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/testing_node/src/Test_Data_Exported/"
files = os.listdir(path)
ate_all = []
filenames = []
for file in files:
    if file.endswith( '.txt'):
        filenames.append(file) #append in list all txt files located in the given path
num_files = len(filenames)/3
print "num_files is: ", num_files

for file in range(1,num_files+1):
    print "File num is: ", file
    df_amcl = pd.read_csv(path+"data_amcl_test"+str(file)+".txt")
    df_gdth = pd.read_csv(path+"data_gdtr_test"+str(file)+".txt") #PENDING fix export_data name to data_gdth
    df_odom = pd.read_csv(path+"data_odom_test"+str(file)+".txt")
    
    df_ate = pd.concat([df_amcl,df_gdth], axis=1) #concat df amcl left, gdth right
    #df_ate = df_ate.drop([" Xamcl_cov"," Yamcl_cov"," Thamcl_cov"], axis=1) #drop unused columns for the moment #PENDING fix espacio en nombres _Xamcl_cov
    print "len inicial df", len(df_ate)
    
    df_ate = df_ate[(~df_ate.isnull()).all(axis=1)] #drop rows with empty values i.e. poses where there is gt but not amcl
    print "len depurada df", len(df_ate)
    
    #Compute pose errors x_error, y_error, th_error, traj_error
    df_ate["x_error"] = df_ate[" Xamcl"] - df_ate[" Xgt"] #PENDING fix espacio en nombres _Xamcl_cov
    df_ate["y_error"] = df_ate[" Yamcl"] - df_ate[" Ygt"]
    df_ate["th_error"] = df_ate[" Thacml"]- df_ate[" Thgt"]
    
    df_ate["traj_error"] = (np.sqrt( (df_ate["x_error"] * df_ate["x_error"]) +  (df_ate["y_error"] * df_ate["y_error"]) ) )
    df_ate["traj_error2"] = df_ate["traj_error"] * df_ate["traj_error"]
    
    #Compute metrics for trajectory error, min, max, mean, std, ate
    minT = round(df_ate["traj_error"].min() ,4)
    maxT = round(df_ate["traj_error"].max() ,4)
    mean = round(df_ate["traj_error"].mean(),4)
    std  = round(df_ate["traj_error"].std() ,4)
    ate  = np.sqrt(df_ate["traj_error2"].mean())
    ate  = round(ate,4)#ATE_RSME = sqrt[mean(error_trayectoria al 2)]    
    print "min is ", minT, "\n", "max is ", maxT, "\n", "mean is ", mean,  "\n", "std is ", std,  "\n", "ATE is ", ate
    
    ######
    #PLOT1 - trajectory error vs. time /trajectory in map (odom, groundtruth, amcl)
    ######
    fig, ax = plt.subplots(1,2)
    fig.set_size_inches(18,8)
    fig.suptitle("AMCL Test "+str(file), fontsize=24)
    
    ax[0].set_title("Trajectory")
    ax[0].plot(df_ate[" Xamcl"], df_ate[" Yamcl"], label="AMCL")
    ax[0].plot(df_ate[" Xgt"], df_ate[" Ygt"], label="Groundtruth")
    ax[0].plot(df_odom[" Xodom"], df_odom[" Yodom"], label="Odometry")
    ax[0].legend(loc='upper left')
    
    ax[1].set_title("Trajectory Error")
    ax[1].plot(df_ate['Seq'], df_ate["traj_error"]) #PENDING convertir tstamps a seg
    ax[1].text(5,4, "ATE: %s\nMean:%s\nStd: %s\nMax: %s\nMin: %s\n" %(ate, mean, std, maxT, minT), fontsize = 10)
    ax[1].set_xlabel("Poses")
    ax[1].set_ylabel("Error")
       
    fig.savefig(path+"amcl_test"+str(file)+"_traj_error.png")
    
    ######
    #PLOT2 - pose error/covariance x,y,th vs. time
    ######
    fig, ax = plt.subplots(3,2)
    fig.set_size_inches(20,16)
    fig.suptitle("AMCL Test "+str(file), fontsize=24)
    
    ax[0,0].plot(df_ate['Seq'], df_ate["x_error"]) 
    ax[0,0].set_title("Pose Error in X")
    ax[1,0].plot(df_ate['Seq'], df_ate["y_error"]) #PENDING convertir tstamps a seg #PENDING ver por que se plotea 2 veces el valor
    ax[1,0].set_title("Pose Error in Y")
    ax[2,0].plot(df_ate['Seq'], df_ate["th_error"]) 
    ax[2,0].set_title("Pose Error in Th")
    ax[0,1].plot(df_ate['Seq'], df_ate[" Xamcl_cov"])  
    ax[0,1].set_title("AMCL cov in X")
    ax[1,1].plot(df_ate['Seq'], df_ate[" Yamcl_cov"]) #PENDING convertir tstamps a seg #PENDING ver por que se plotea 2 veces el valor
    ax[1,1].set_title("AMCL cov in Y")
    ax[2,1].plot(df_ate['Seq'], df_ate[" Thamcl_cov"]) 
    ax[2,1].set_title("AMCL cov in Th")
    
    fig.savefig(path+"amcl_test"+str(file)+"_pose_error_cov.png")
    ate_all.append([ate])


######
#PLOT3 - all tests statistics
######
df_ate_all   = pd.DataFrame(ate_all, columns=["ATE"])
mean_ate_all = round(df_ate_all["ATE"].mean(),4)
print df_ate_all
#a-trayectorias encimadas, traj. error encimados, ATE barras
fig, ax = plt.subplots(1,3)
fig.set_size_inches(18,6)
fig.suptitle("AMCL Tests Summary", fontsize=24)

threshold = mean_ate_all
df_ate_all['ATE'].plot.bar(x='Test',y='ATE',ax=ax[0]) 
ax[0].set_title("ATE")
ax[0].plot([0., len(df_ate_all)], [threshold, threshold], "k--")
ax[0].text(3,3.1, "Mean:%s\n" % (mean_ate_all), fontsize = 10)

ax[1].set_title("RPE")

ax[2].set_title("Trajectory Error")

fig.savefig(path+"amcl_tests_summary.png")