/**
 * Author: Akhil Kumar Nagariya
 * RRC, IIIT Hyderabad
 * EKF Localization for Turtlebot 
 **/
#include "ekf_turtlebot/Ekf.h"
#include <sstream>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <boost/ref.hpp>
#include <cstdio>
using Eigen::MatrixXd;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization");
    

    ros::NodeHandle n;
 
    //publishes turtlebot pose with covariance
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseWithCovariance>("turtle_pose", 1);

    //publish command to turtlebot
    ros::Publisher turtle_command = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",10);
    //subsribe to sensor msg
    

    ros::Rate loop_rate(1);

    Matrix3d initial_covar;

    //enter initial covariance here
    initial_covar<<0.6, 0, 0,
                   0,0.6 , 0,
                   0, 0, 0.01;

    //enter initial position here
    Vector3d initial_position(0,0,0);

    //---> call the constructor with landmark position, file initial position and covariance of the robot 
    //replace /home/akhil/a.txt with your landmark file (provide absolute path)
    
    Ekf ekf("/home/akhil/a.txt",initial_position,initial_covar);

  
  //  ros::Subscriber sub = n.subscribe("visualization_marker",100,sensorCallback);
    while(ros::ok()){
        cout<<"please enter motion commands:(v,w,del_t)"<<endl;
        float v,w,del_t;
        cin>>v>>w>>del_t;
        
        ros::Time then = ros::Time::now();
        
        //publish commands to turtlebot
        ros::Rate loop_rate_command(100);
        geometry_msgs::Twist twist;
        twist.linear.x = v;
        twist.linear.z = w;
        while(ros::Time::now() - then < ros::Duration(del_t)){
            turtle_command.publish(twist);
            loop_rate_command.sleep();
        }
            
        //do motion update
        ekf.motionUpdate(v,w,del_t);
        //If there is a sensor measurement then update the measurements
        ros::spinOnce(); //ros::spinOnce() calls all the subscribers and the publihser 
                         //of the node in saperate threads
                         //in this case we only have one subscriber
                         //sensorCallback which will update the current
                         //measurement  
        //check if we have a new sensor measurement and call sensorUpdate
        if(ekf.sensor_update_flag){
            ROS_INFO("performing sensor update");
            ekf.sensorUpdate();
            ros::spinOnce(); //called to publish pose after sensorUpdate
            ekf.sensor_update_flag = false;
        }
                          
    }


    return 0;
}
