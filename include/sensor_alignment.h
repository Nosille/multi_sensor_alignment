/** ROS node 

/* 

Copyright (c) 2017

*/

#ifndef MULTI_SENSOR_ALIGNMENT_H
#define MULTI_SENSOR_ALIGNMENT_H

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_representation.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/visualization/pcl_visualizer.h>

#include <cv_bridge/cv_bridge.h>

namespace Multi_Sensor_Alignment
{
  class SourceCloud
  {
    public:
      SourceCloud(ros::NodeHandle &node_handle, const std::string &input_topic)
      {
        cloud_in_topic = input_topic;
      }

      std::string cloud_in_topic;

      ros::Subscriber cloud_sub;

      std::list<pcl::PCLPointCloud2> cloud_buffer;
      std::list<tf2::Transform> transform_buffer;
      std::list<std_msgs::Header> header_buffer;

  };

  class SourceCamera
  {
    public:
      SourceCamera(ros::NodeHandle &node_handle, const std::string &image_topic, const std::string &info_topic)
      {
        cameraImage_topic = image_topic;
        cameraInfo_topic = info_topic;

        cameraImage_sub = new message_filters::Subscriber<sensor_msgs::Image>(node_handle, cameraImage_topic, 10);
        cameraInfo_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(node_handle, cameraInfo_topic, 10);
        cameraSync  = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo>(*cameraImage_sub, *cameraInfo_sub, 10);
        frameID = "";

        distortion_model = "plumb_bob";
        distortion_coefficients = cv::Mat(1, 5, CV_64F);
        camera_instrinsics = cv::Mat(3, 3, CV_64F);
        projection_matrix = cv::Mat(3, 4, CV_64F);

        fx = 0;
        fy = 0;
        cx = 0;
        cy = 0;
      }

      ~SourceCamera()
      {
        // delete cameraImage_sub;
        // delete cameraInfo_sub;
        // delete cameraSync;
      }

      std::string cameraImage_topic, cameraInfo_topic;

      message_filters::Subscriber<sensor_msgs::Image>* cameraImage_sub;
      message_filters::Subscriber<sensor_msgs::CameraInfo>* cameraInfo_sub;
      message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> *cameraSync;
      
      std::string distortion_model;
      cv::Mat distortion_coefficients;
      cv::Mat camera_instrinsics;
      cv::Mat projection_matrix;
      cv_bridge::CvImage image;

      std::string frameID;
      geometry_msgs::TransformStamped transform;

      float fx;
      float fy;
      float cx;
      float cy;
  };

  // boost::shared_ptr<pcl::visualization::PCLVisualizer>
  // simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
  // {
  //   // --------------------------------------------
  //   // -----Open 3D viewer and add point cloud-----
  //   // --------------------------------------------
  //   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //   viewer->setBackgroundColor (0, 0, 0);
  //   viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  //   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //   //viewer->addCoordinateSystem (1.0);
  //   viewer->initCameraParameters ();
  //   return (viewer);
  // };
  
} // namespace Multi_Sensor_Alignment

#endif  // Cloud_Fusion_H

