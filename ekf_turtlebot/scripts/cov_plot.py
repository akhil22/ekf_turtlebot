#!/usr/bin/env python
import rospy
import PyKDL
from visualization_msgs.msg import Marker, MarkerArray
import visualization_msgs
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovariance
import numpy
from numpy import concatenate
import math 
#syntetic 3-D Gaussian probabilistic model for sampling
covMatModel =[[1, 0, 0],[0, 1, 0],[0, 0, 1]]
meanModel =[0, 0, 0]
 
#used to paint the autovectors
def posecallback(data):
    meanModel[0]=data.pose.position.x
    meanModel[1]=data.pose.position.y
    meanModel[2]=data.pose.position.z
    covMatModel[0][0] = data.covariance[0]
    covMatModel[0][1] = data.covariance[1]
   # covMatModel[0][2] = data.covariance[2]
    covMatModel[0][2] = 0
    covMatModel[1][0] = data.covariance[3]
    covMatModel[1][1] = data.covariance[4]
   # covMatModel[1][2] = data.covariance[5]
    covMatModel[1][2] = 0 
   # covMatModel[2][0] = data.covariance[6]
   # covMatModel[2][1] = data.covariance[7]
    covMatModel[2][0] = 0
    covMatModel[2][1] = 0
    covMatModel[2][2] = 0
   # covMatModel[2][2] = data.covariance[8]
    covMat = covMatModel 
    mean = meanModel
 
    #painting the gaussian ellipsoid marker
    marker = Marker ()
    marker.header.frame_id ="/map";
    marker.header.stamp = rospy.Time.now ()
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs.msg.Marker.SPHERE
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.pose.position.x = mean[0]
    marker.pose.position.y = mean[1]
    marker.pose.position.z = mean[2]
 
    #getting the distribution eigen vectors and values
    (eigValues,eigVectors) = numpy.linalg.eig (covMat)
 
    #painting the eigen vectors
    id=1
    for v in eigVectors:
        m=markerVector(id, v*eigValues[id-1], mean)
        id=id+1
        points_pub.publish(m)
 
    #building the rotation matrix
    eigx_n=PyKDL.Vector(eigVectors[0,0],eigVectors[0,1],eigVectors[0,2])
    eigy_n=-PyKDL.Vector(eigVectors[1,0],eigVectors[1,1],eigVectors[1,2])
    eigz_n=PyKDL.Vector(eigVectors[2,0],eigVectors[2,1],eigVectors[2,2])
    eigx_n.Normalize()
    eigy_n.Normalize()
    eigz_n.Normalize()
    rot = PyKDL.Rotation (eigx_n,eigy_n,eigz_n)
    rot = PyKDL.Rotation.RPY(0,0,math.atan2(eigVectors[0,1],eigVectors[0,0]));
    quat = rot.GetQuaternion ()
 
    #painting the Gaussian Ellipsoid Marker
    marker.pose.orientation.x =quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]
    marker.scale.x = eigValues[0]*2
    marker.scale.y = eigValues[1]*2
    marker.scale.z =eigValues[2]*2
 
    marker.color.a = 0.5
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
 
    gauss_pub.publish (marker)
    rospy.loginfo("hrere");
def markerVector(id,vector,position):
    marker = Marker ()
    marker.header.frame_id = "map";
    marker.header.stamp = rospy.Time.now ()
    marker.ns = "my_namespace2";
    marker.id = id;
    marker.type = visualization_msgs.msg.Marker.ARROW
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.scale.x=0.02
    marker.scale.y=0.02
    marker.scale.z=0.02
    marker.color.a= 1.0
    marker.color.r = 0.33*float(id)
    marker.color.g = 0.33*float(id)
    marker.color.b = 1*float(id)
    (start,end)=(Point(),Point())
 
    start.x = position[0]
    start.y = position[1]
    start.z = position[2]
    end.x=start.x+vector[0]
    end.y=start.y+vector[1]
    end.z=start.z+vector[2]
 
    marker.points.append(start)
    marker.points.append(end)
    print str(marker)
    return marker
 
rospy.init_node ('markersample', anonymous = True)
points_pub = rospy.Publisher ("covariance_markers", visualization_msgs.msg.Marker)
gauss_pub = rospy.Publisher ("gaussian", visualization_msgs.msg.Marker)
pose_pub = rospy.Subscriber("turtle_pose", PoseWithCovariance, posecallback)
while not rospy.is_shutdown ():
    syntetic_samples = None
 
    #painting all the syntetic points
    for i in xrange (10, 5000):
        p = numpy.random.multivariate_normal (meanModel, covMatModel)
        if syntetic_samples == None:
            syntetic_samples =[p]
        else:
            syntetic_samples = concatenate ((syntetic_samples,[p]), axis = 0)
 
 
    #calculating Gaussian parameters
    syntetic_samples = numpy.array (syntetic_samples)
    covMat = numpy.cov (numpy.transpose (syntetic_samples))
    mean = numpy.mean ([syntetic_samples[: , 0], syntetic_samples[: , 1], syntetic_samples[:, 2]], axis = 1)
    covMat = covMatModel 
    mean = meanModel
 
    #painting the gaussian ellipsoid marker
    marker = Marker ()
    marker.header.frame_id ="/map";
    marker.header.stamp = rospy.Time.now ()
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs.msg.Marker.SPHERE
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.pose.position.x = mean[0]
    marker.pose.position.y = mean[1]
    marker.pose.position.z = mean[2]
 
    #getting the distribution eigen vectors and values
    (eigValues,eigVectors) = numpy.linalg.eig (covMat)
 
    #painting the eigen vectors
    id=1
    for v in eigVectors:
        m=markerVector(id, v*eigValues[id-1], mean)
        id=id+1
        points_pub.publish(m)
 
    #building the rotation matrix
    eigx_n=PyKDL.Vector(eigVectors[0,0],eigVectors[0,1],eigVectors[0,2])
    eigy_n=-PyKDL.Vector(eigVectors[1,0],eigVectors[1,1],eigVectors[1,2])
    eigz_n=PyKDL.Vector(eigVectors[2,0],eigVectors[2,1],eigVectors[2,2])
    eigx_n.Normalize()
    eigy_n.Normalize()
    eigz_n.Normalize()
    rot = PyKDL.Rotation (eigx_n,eigy_n,eigz_n)
    quat = rot.GetQuaternion ()
 
    #painting the Gaussian Ellipsoid Marker
    marker.pose.orientation.x =quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]
    marker.scale.x = eigValues[0]*2
    marker.scale.y = eigValues[1]*2
    marker.scale.z =eigValues[2]*2
 
    marker.color.a = 0.5
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    rospy.spin()
 
