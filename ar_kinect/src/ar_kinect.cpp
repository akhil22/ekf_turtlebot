/*
 *  Multi Marker Pose Estimation using ARToolkit & Kinect
 *  Copyright (C) 2010, CCNY Robotics Lab, 2011 ILS Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  William Morris <morris@ee.ccny.cuny.edu>
 *  Gautier Dumonteil <gautier.dumonteil@gmail.com>
 *  http://robotics.ccny.cuny.edu
 * 
 *  Michael Ferguson <ferguson@cs.albany.edu>
 *  http://robotics.ils.albany.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include "ar_kinect/ar_kinect.h"
#include "ar_kinect/object.h"



int main (int argc, char **argv)
{
  ros::init (argc, argv, "ar_kinect");
  ros::NodeHandle n;
  ar_pose::ARPublisher ar_kinect (n);
  ros::spin ();
  return 0;
}

namespace ar_pose
{
  tf::Transform tfFromEigen(Eigen::Matrix4f trans)
  {
    tf::Matrix3x3 btm;
    btm.setValue(trans(0,0),trans(0,1),trans(0,2),
               trans(1,0),trans(1,1),trans(1,2),
               trans(2,0),trans(2,1),trans(2,2));
    tf::Transform ret;
    ret.setOrigin(tf::Vector3(trans(0,3),trans(1,3),trans(2,3)));
    ret.setBasis(btm);
    return ret;
  }

  pcl::PointXYZRGB makeRGBPoint( float x, float y, float z )
  {
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y; 
    p.z = z;
    return p;
  }

  ARPublisher::ARPublisher (ros::NodeHandle & n):n_ (n), configured_(false)
  {
    std::string path;
    std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
    ros::NodeHandle n_param ("~");
    XmlRpc::XmlRpcValue xml_marker_center;

    // **** get parameters

    if (!n_param.getParam ("publish_tf", publishTf_))
      publishTf_ = true;
    ROS_INFO ("\tPublish transforms: %d", publishTf_);

    if (!n_param.getParam ("publish_visual_markers", publishVisualMarkers_))
      publishVisualMarkers_ = true;
    ROS_INFO ("\tPublish visual markers: %d", publishVisualMarkers_);

    if (!n_param.getParam ("threshold", threshold_))
      threshold_ = 100;
    ROS_INFO ("\tThreshold: %d", threshold_);

    if (!n_param.getParam ("marker_pattern_list", path)){
      sprintf(pattern_filename_, "%s/data/objects_kinect", package_path.c_str());
    }else{
      sprintf(pattern_filename_, "%s", path.c_str());
    }    
    ROS_INFO ("Marker Pattern Filename: %s", pattern_filename_);

    if (!n_param.getParam ("marker_data_directory", path)){
      sprintf(data_directory_, "%s", package_path.c_str());
    }else{
      sprintf(data_directory_, "%s", path.c_str());
    }    
    ROS_INFO ("Marker Data Directory: %s", data_directory_);

    // **** subscribe

    configured_ = false;
    cloud_sub_ = n_.subscribe(cloudTopic_, 1, &ARPublisher::getTransformationCallback, this);

    // **** advertise 

    arMarkerPub_ = n_.advertise < ar_pose::ARMarkers > ("ar_pose_markers",0);
    if(publishVisualMarkers_)
    {
		rvizMarkerPub_ = n_.advertise < visualization_msgs::Marker > ("visualization_marker", 0);
    }
  }

  ARPublisher::~ARPublisher (void)
  {
    arVideoCapStop ();
    arVideoClose ();
  }

  /* 
   * Setup artoolkit
   */
  void ARPublisher::arInit ()
  {
    arInitCparam (&cam_param_);
    ROS_INFO ("*** Camera Parameter ***");
    arParamDisp (&cam_param_);

    // load in the object data - trained markers and associated bitmap files
    if ((object = ar_object::read_ObjData (pattern_filename_, data_directory_, &objectnum)) == NULL)
      ROS_BREAK ();
    ROS_DEBUG ("Objectfile num = %d", objectnum);

    sz_ = cvSize (cam_param_.xsize, cam_param_.ysize);
    capture_ = cvCreateImage (sz_, IPL_DEPTH_8U, 4);
    configured_ = true;
  }

  /* 
   * One and only one callback, now takes cloud, does everything else needed. 
   */
  void ARPublisher::getTransformationCallback (const sensor_msgs::PointCloud2ConstPtr & msg)
  {
    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
    ARUint8 *dataPtr;
    ARMarkerInfo *marker_info;
    int marker_num;
    int i, k, j;

    /* do we need to initialize? */
    if(!configured_)
    {
      if(msg->width == 0 || msg->height == 0)
      {
        ROS_ERROR ("Deformed cloud! Size = %d, %d.", msg->width, msg->height);
        return;
      }

      cam_param_.xsize = msg->width;
      cam_param_.ysize = msg->height;

      cam_param_.dist_factor[0] = msg->width/2;         // x0 = cX from openCV calibration
      cam_param_.dist_factor[1] = msg->height/2;        // y0 = cY from openCV calibration
      cam_param_.dist_factor[2] = 0;                    // f = -100*k1 from CV. Note, we had to do mm^2 to m^2, hence 10^8->10^2
      cam_param_.dist_factor[3] = 1.0;                  // scale factor, should probably be >1, but who cares...
      
      arInit ();
    }

    /* convert cloud to PCL & PCLPointCloud2 */
    PointCloud cloud;    
    pcl::PCLPointCloud2 cloud_2;    
    pcl_conversions::toPCL(*msg, cloud_2);
    pcl::fromPCLPointCloud2(cloud_2, cloud);
    
    /* get an OpenCV image from the cloud */
    pcl::PCLImage pcl_image;
    pcl::toPCLPointCloud2(cloud_2, pcl_image); 
    pcl_conversions::moveFromPCL(pcl_image, *image_msg);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR ("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str ());
    }
    dataPtr = (ARUint8 *) cv_ptr->image.ptr();

    /* detect the markers in the video frame */
    if (arDetectMarkerLite (dataPtr, threshold_, &marker_info, &marker_num) < 0)
    {
      argCleanup ();
      return;
    }
 
    arPoseMarkers_.markers.clear ();
    /* check for known patterns */
    for (i = 0; i < objectnum; i++)
    {
      k = -1;
      for (j = 0; j < marker_num; j++)
      {
        if (object[i].id == marker_info[j].id)
        {
          if (k == -1)
            k = j;
          else                  // make sure you have the best pattern (highest confidence factor)
          if (marker_info[k].cf < marker_info[j].cf)
            k = j;
        }
      }
      if (k == -1)
      {
        object[i].visible = 0;
        continue;
      }
      
      /* create a cloud for marker corners */
      int d = marker_info[k].dir;
      PointCloud marker;
      marker.push_back( cloud.at( (int)marker_info[k].vertex[(4-d)%4][0], (int)marker_info[k].vertex[(4-d)%4][1] ) ); // upper left
      marker.push_back( cloud.at( (int)marker_info[k].vertex[(5-d)%4][0], (int)marker_info[k].vertex[(5-d)%4][1] ) ); // upper right
      marker.push_back( cloud.at( (int)marker_info[k].vertex[(6-d)%4][0], (int)marker_info[k].vertex[(6-d)%4][1] ) ); // lower right
      marker.push_back( cloud.at( (int)marker_info[k].vertex[(7-d)%4][0], (int)marker_info[k].vertex[(7-d)%4][1] ) );

      /* create an ideal cloud */
      double w = object[i].marker_width;
      PointCloud ideal;
      ideal.push_back( makeRGBPoint(-w/2,w/2,0) );
      ideal.push_back( makeRGBPoint(w/2,w/2,0) );
      ideal.push_back( makeRGBPoint(w/2,-w/2,0) );
      ideal.push_back( makeRGBPoint(-w/2,-w/2,0) );

      /* get transformation */
      Eigen::Matrix4f t;
      TransformationEstimationSVD obj;
      obj.estimateRigidTransformation( marker, ideal, t );

      
      /* get final transformation */
      tf::Transform transform = tfFromEigen(t.inverse());
   
      // any(transform == nan)
      tf::Matrix3x3  m = transform.getBasis();
      tf::Vector3    p = transform.getOrigin();
      bool invalid = false;
      for(int i=0; i < 3; i++)
        for(int j=0; j < 3; j++)
          invalid = (invalid || isnan(m[i][j]) || fabs(m[i][j]) > 1.0);

      for(int i=0; i < 3; i++)
          invalid = (invalid || isnan(p[i]));
       

      if(invalid)
        continue; 

      /* publish the marker */
      ar_pose::ARMarker ar_pose_marker;
      ar_pose_marker.header.frame_id = msg->header.frame_id;
      ar_pose_marker.header.stamp = msg->header.stamp;
      ar_pose_marker.id = object[i].id;

      ar_pose_marker.pose.pose.position.x = transform.getOrigin().getX();
      ar_pose_marker.pose.pose.position.y = transform.getOrigin().getY();
      ar_pose_marker.pose.pose.position.z = transform.getOrigin().getZ();

      ar_pose_marker.pose.pose.orientation.x = transform.getRotation().getAxis().getX();
      ar_pose_marker.pose.pose.orientation.y = transform.getRotation().getAxis().getY();
      ar_pose_marker.pose.pose.orientation.z = transform.getRotation().getAxis().getZ();
      ar_pose_marker.pose.pose.orientation.w = transform.getRotation().getW();

      ar_pose_marker.confidence = marker_info->cf;
      arPoseMarkers_.markers.push_back (ar_pose_marker);

      /* publish transform */
      if (publishTf_)
      {
	    broadcaster_.sendTransform(tf::StampedTransform(transform, msg->header.stamp, msg->header.frame_id, object[i].name));
      }

      /* publish visual marker */

      if (publishVisualMarkers_)
      {
        tf::Vector3 markerOrigin (0, 0, 0.25 * object[i].marker_width * AR_TO_ROS);
        tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin);
        tf::Transform markerPose = transform * m; // marker pose in the camera frame

        tf::poseTFToMsg (markerPose, rvizMarker_.pose);

        rvizMarker_.header.frame_id = msg->header.frame_id;
        rvizMarker_.header.stamp = msg->header.stamp;
        rvizMarker_.id = object[i].id;

        rvizMarker_.scale.x = 1.0 * object[i].marker_width * AR_TO_ROS;
        rvizMarker_.scale.y = 1.0 * object[i].marker_width * AR_TO_ROS;
        rvizMarker_.scale.z = 0.5 * object[i].marker_width * AR_TO_ROS;
        rvizMarker_.ns = "basic_shapes";
        rvizMarker_.type = visualization_msgs::Marker::CUBE;
        rvizMarker_.action = visualization_msgs::Marker::ADD;
        switch (i)
        {
          case 0:
            rvizMarker_.color.r = 0.0f;
            rvizMarker_.color.g = 0.0f;
            rvizMarker_.color.b = 1.0f;
            rvizMarker_.color.a = 1.0;
            break;
          case 1:
            rvizMarker_.color.r = 1.0f;
            rvizMarker_.color.g = 0.0f;
            rvizMarker_.color.b = 0.0f;
            rvizMarker_.color.a = 1.0;
            break;
          default:
            rvizMarker_.color.r = 0.0f;
            rvizMarker_.color.g = 1.0f;
            rvizMarker_.color.b = 0.0f;
            rvizMarker_.color.a = 1.0;
        }
        rvizMarker_.lifetime = ros::Duration ();

        rvizMarkerPub_.publish (rvizMarker_);
        ROS_DEBUG ("Published visual marker");
      }
    }
    arMarkerPub_.publish (arPoseMarkers_);
    ROS_DEBUG ("Published ar_multi markers");
  }

}                               // end namespace ar_pose
