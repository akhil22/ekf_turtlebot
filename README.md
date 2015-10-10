This folder contains three packages that you need to install to run the localization. Please follow these instructions to install them:

ar_tool,ar_kinect : 
             1. to install ar_tool and ar_kinect you need a rosbuild workspace please type following commands in your terminal to 
             create it.

             $ mkdir ~/rosbuild_ws
	     $ cd ~/rosbuild_ws
	     $ rosws init . /opt/ros/<version>

	     replace version by your ros version (indigo,groovy,hydro....)
	  2. now copy ar_tool and ar_kinect in rosbuild_ws directory and use rosmake to build this stack-
            
             $ cp -r ar_tool ~/rosbuild_ws 
             $ cp -r ar_kinect ~/rosbuild_ws
             $ source ~/rosbuild_ws/setup.bash
             $ rospack profile
             $ rosmake ar_tool ar_kinect

            if above process doesn't work because you have some missing packages (ros or other 3rd party) then please install them(covered 
            in the class). 

ekf_turtlebot:
           This is the main localization package. It is a catkin package, process to install catkin packages is already covered in ros
           tutorial class.

ar_tool and ar_kinect packages are used to detect Augmented Reality tags (AR tags) by kinect in the environment. In this project these tags are used as landmarks. The environment will contain several of these tags. For visualizaion we will use rviz, copy the file default.rviz 
in the directory ~/.rviz(if the directory doesn't exist then create it) this will load the default configuration when we start rviz.

Once you have installed these packages you have to modify ekf_turtlebot package to run the localization please follow the README.txt file inside ekf_turtlebot package for further instructions. 
 	    
