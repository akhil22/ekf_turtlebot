Project Name: ekf_turtlebot
Description : Implements EKF localization on turtlebot
Author: Akhil Kumar Nagariya, RRC, IIIT Hyderabad
Algorithm:Ekf_localiazation_known_correspondence Page 204 probabilistic Robotics book 

You are expected to follow these steps to successfully complete this assignment. We are assuming that we know the identity of the landmark from which the measurement is coming so the measurement vector is 1x2 (q,phi) and not 1x3(q,phi,s_i) this will accordingly change the size of S,H and K in sensorUpdate function. 

1. This project contains following files. You are also provided with instructions if anything needs to be changed in these files.  
   
   localization.cpp ---> Reads the motion commands and executes them on turtlebot also creates an Ekf object which contains all the data and 
                         functions you need to implement EKF. You don't have to change anything in this file except the file name which  	                  contains position of the landmarks(AR tags). change the file name after the comment starting with //---> (line 89). 	                      The file format is as following-
                         
                         n
                         x1 y1
                         x2 y2
                         x3 y3
                         .  .
                         .  .
                         xn yn 
                        
                         Where n is the total number of landmarks and (xi,yi) is the position of ith landmark. 
                         You will have to change this file according to your environment 
   
   include/Ekf.h    ---> Implements Ekf localization You have to go through this file and change it according to the comments if they     	                   start with //--->. 
                         
                         Please go through this file from line 1 as it contains details on how you are supposed to perform matrix operation 
                         in first few comments.

2. After updating Ekf.h you have to come to the lab to test your code on turtlebot but before this You can check if the motion update is working or not by running the following commands.
   
   $ rosrun rviz rviz --> start rviz for visualization
   
   $rosrun ekf_turtlebot cov_plot.py ---> plots covariance matrix
   
   $ rosrun ekf_turtlebot ekf_turtlebot_node ---> runs the localization

   This will ask you to enter linear velocity , angular velocity and time after this it performs motion update and sensor update.  
   Enter 0.1 0.1 1 this will do the motion update and you will see a covariance ellipse in rviz, The code will ask you to enter the     
   commands again enter the same values and if your motion update function is correct the size of error ellipse will increase.
 
3. Now the next step is to run the localization on turtlebot. After bringing up the turtlebot run the following commands in your terminal.   this will start the landmark detection using kinect (assuming you have already installed ar_tools and ar_kinect packages)
   
   $ roslaunch ar_kinect ar_kinect.launch
 
   For visualization run the following command, this will start rviz.

   $ rosrun rviz rviz 

   To plot the covariance elipse run the following command in your terminal.

   $rosrun ekf_turtlebot cov_plot.py

   To start localization run following command in your terminal

   $ rosrun ekf_turtlebot ekf_turtlebot_node

   This will ask you to enter linear velocity , angular velocity and time after this it publishes the command to turtlebot and then 
   performs motion update and sensor update.

 
