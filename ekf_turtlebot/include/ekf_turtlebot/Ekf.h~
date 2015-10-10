/**
 * Author: Akhil Kumar Nagariya
 * RRC IIIT Hyderabad
 * Implements algo:
 * EKF_localization_known_correspondences(Mu_t-1,Sigme_t-1,Mu_t,Z_t,C_t,M) from
 * probabilistic Robotics book Page 204
 * Note: code uses Eigen for linear algebra operations go through this header and
 * you will have an idea of how it works for example to create a matrix use
 * MatrixXd or Matrix3d for unknown dimension or 3x3 matrix respectively
 * to add/sub Matrix/Vectors A and B use A +/- B
 * to multiply Matrix/Vectors A and B use A*B
 * to divide/multiply a Matrix A by scalar b use A/b or A*b
 * A.transpose() ==> Transpose of A
 * A.inverse() ==> inverse of A
 * To create a vector v=[v1,v2] use vector2d v(v1,v2);
 * To create a matrix H = [a11,a12,a13;a21,a22,a23;a31,a32,a32] use MatrixXd H(3,3); H<<a11,a12,a13,a21,a22,a23,a31,a32,a33; 
 * for more detailed tutorial (only for interested people / if you have any
 * errors during compilation otherwise you don't need more then above mentioned operations)
 * http://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html
 * */

#include "ros/ros.h"                                                                                          
#include "std_msgs/String.h"   
#include<Eigen/Dense>
#include<string>
#include<fstream>
#include<iostream>
#include<cmath>
#include <geometry_msgs/PoseWithCovariance.h>
#include<visualization_msgs/Marker.h>
#include "boost/bind.hpp" 
#include <tf/transform_broadcaster.h>
using namespace Eigen;
using namespace std;
class Ekf{
    public: 
        Ekf(char*,Vector3d,Matrix3d);              //constructor call to initialize the filter
        void motionUpdate(float,float,float);      //performs motion update pass v,w,t(executes v,w for time t) 
        void sensorUpdate();                       //performs sensor update pass r_i,phi_i,s_i
                                                   //r_i = r for s_i th landmark
                                                   //phi_i = phi for s_i th landmark
                                                   //s_i = identity of the landmark
                                                   //(ex 0,1 ....)
        
        Vector3d current_mean;                    //refers to Mu_t
        Matrix3d current_covar;                   //refers to Sigme_t
        float current_measurement_q;              //refers to the q of current landmark measurement   
        float current_measurement_phi;            //refers to the phi of current landmark measurement
        float current_measurement_si;             //refers to the signature of current landmark measurement
        bool sensor_update_flag;                  //used to check if a landmark is detected 
        ros::Publisher pose_pub;                  //publisher for publishing pose
        ros::Subscriber sensor_sub;               //subscriber to get measurements from kinect
        void publishPose();                       //publishes pose data
        void sensorCallback(const visualization_msgs::Marker::ConstPtr&);  //updates current measurement
    private:
        MatrixXd landmarks_;                       //refers to the landmarks 
        string landmark_file_;                     //name of the file containing landmark positions
        int num_landmarks_;

        
        void readLandmarks();
                                                   //All the symbols below corresond to the mentioned algo. 
        Matrix2d M;                                //control noise covariance.  
        Matrix2d Q;                                //Measurement noise covariance note that it is 2x2 and not 3x3 this is explained later    

                                               
};

Ekf::Ekf(char* landmark_file,Vector3d mean,Matrix3d covar){
    landmark_file_ = landmark_file;
    current_mean = mean;
    current_covar = covar;
    readLandmarks();

    //---> initialize control noise covariance update M here:
    M << 0, 0,
         0, 0;

    //---> initialize measurement noise covariance update Q here note that it is 2x2
    //not 3x3 because mesurement vector is 2x1 we are not considering the
    //signature in the measurement vectors accordingly size of H,S and K will
    //change:
    Q <<0, 0, 
        0, 0;

    //publish pose with covariance
    ros::NodeHandle nh;
    pose_pub = nh.advertise<geometry_msgs::PoseWithCovariance>("turtle_pose",10);
    sensor_sub = nh.subscribe("visualization_marker",1, &Ekf::sensorCallback, this);
    publishPose();
    
} 


//you can ignore this function it just publishes pose data on turtle_pose topic 
void Ekf::publishPose(){
    geometry_msgs::PoseWithCovariance msg;                                                                                                  
    msg.pose.position.x = current_mean(0);                                                                                              
    msg.pose.position.y = current_mean(1);                                                                                              
    ros::Rate loop_rate(10);
    msg.covariance[0] = current_covar(0,0);                                                                                             
    msg.covariance[1] = current_covar(0,1);                                                                                             
    msg.covariance[2] = current_covar(0,2);                                                                                             
    msg.covariance[3] = current_covar(1,0);                                                                                             
    msg.covariance[4] = current_covar(1,1);                                                                                             
    msg.covariance[5] = current_covar(1,2);                                                                                             
    msg.covariance[6] = current_covar(2,0);                                                                                             
    msg.covariance[7] = current_covar(2,1);                                                                                             
    msg.covariance[8] = current_covar(2,2);
    pose_pub.publish(msg);                                                                                                           
    loop_rate.sleep();                                                                                                                  
    pose_pub.publish(msg); 
    loop_rate.sleep();                                                                                                                  
    pose_pub.publish(msg); 
    loop_rate.sleep();                                                                                                                  
    pose_pub.publish(msg); 
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(current_mean(0), current_mean(1), 0));
    tf::Quaternion q;
    q.setRPY(0, 0, current_mean(2));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera_link"));
}
void Ekf::motionUpdate(float v,float w,float del_t){
    //refer to the mentioned algo for details
    //(current_mean(0),current_mean(1)) is the current mean postion of robot
    //current_covar is the current error covariance the robot
    
    float theta = current_mean(2);
    //--->calculate G and update here 
    Matrix3d G;

    //--->calculate V and update here 
    MatrixXd V(3,2);

    //--->update mean Mu_t here
    
    //--->update covarince Sigma_t here. To take transpose of G or V use
    //G.transpose() or V.transpose() respectively
    stringstream ss;
    ss<<endl<<"after motion update mean = "<<endl<<current_mean<<endl<<"covariance = "<<endl<<current_covar<<endl;
    ROS_INFO("%s",ss.str().c_str());      
    publishPose();
    
}
void Ekf::sensorUpdate(){
    //refer to the mentioned algo for details

    //most recent measurement
    float r_i = current_measurement_q;
    float phi_i = current_measurement_phi;
    int s_i = current_measurement_si;
    
    //caculate z_sensor here
    Vector2d z_sensor(r_i,phi_i); //note that this is 2x1 and not 3x1 because of the assumption that signature is always correct.
    
    //--->calculate q here
    float q = 0;    //(landmark_(0,s_i),landmark_(1,s_i)) is the actual postion of s_i th landmark
                    //(current_mean(0),current_mean(1)) is the mean postion of robot use
                    //these two to calculate q and z_estimated
    //calculate z_estimated here
    Vector2d z_estimated(sqrt(q),atan2(landmarks_(1,s_i) - current_mean(1),landmarks_(0,s_i) - current_mean(0)) - current_mean(2));
    
    //--->caculate H here, beacause z is 2x1 H is 2x3 and not 3x3
    MatrixXd H(2,3);
    H<<0, 0, 0,
       0, 0, 0;

    //--->calculate S here, to take transpose of H use H.transpose()
    MatrixXd S;
    
    //--->calculate K here, to take the inverse of S use s.inverse()
    MatrixXd K;

    //--->update mean Mu_t here (current_mean is Mu_t)
    
    //--->update Covariance Sigma_t here(current_covar is Sigma_t)
    
    MatrixXd I = MatrixXd::Identity(3,3); //you will need this to update Sigma_t

    stringstream ss;                                                                                          

    ss<<endl<<"after sensor update mean = "<<endl<<current_mean<<endl<<"covariance = "<<endl<<current_covar<<endl;

    ROS_INFO("%s",ss.str().c_str());
    publishPose(); 
}
void Ekf::sensorCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    float x = msg->pose.position.z;
    float y = -1*msg->pose.position.x;
    int s_i = msg->id;
    
    //--->calculate q,phi from the above values 
    float q = 0;  //(x,y) are the measurements of the position of landmark s_i, (current_mean(0),current_mean(1)) is the mean position of
                  //robot use these two to calculate q and phi  
                  
    float phi = 0;
    ROS_INFO("sensor measurement from landmark number %d (r = %f and phi = %f)",s_i,q,phi);
    current_measurement_q = q;
    current_measurement_phi = phi;
    current_measurement_si = s_i;
    sensor_update_flag = true;

}


//you can ingnore this function it just reads the position of landmarks from the landmark file
void Ekf::readLandmarks(){
    ROS_INFO("reading Landmarks file");

    //reading file 
    ifstream filep(landmark_file_.c_str());
    if(!filep){
        ROS_WARN("landmarks position file doesnot exist");
        return;
    }
    string line;
    getline(filep,line);
    sscanf(line.c_str(),"%d",&num_landmarks_);

    ROS_INFO("reading position of %d landmarks",num_landmarks_);
    
    landmarks_.resize(2,num_landmarks_);
    for(int i = 0;i < num_landmarks_;i++){
        if(getline(filep,line)){
            sscanf(line.c_str(),"%lf %lf",&landmarks_(0,i),&landmarks_(1,i));
            ROS_INFO("position of %d landmark = (%lf,%lf)",i,landmarks_(0,i),landmarks_(1,i));
        }
        else{
            ROS_WARN("file contains less landmarks then specified, setting number of landmarks = %d",i);
            num_landmarks_ = i;
            break;
        }
    }
}
                    

        
