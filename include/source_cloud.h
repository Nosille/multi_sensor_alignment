/** ROS node 

/* 

Copyright (c) 2017

*/

#ifndef SOURCE_CLOUD_H
#define SOURCE_CLOUD_H

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <image_transport/image_transport.h>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_representation.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/visualization/pcl_visualizer.h>

#include <cv_bridge/cv_bridge.h>

namespace ERDC
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

} // namespace Multi_Sensor_Alignment

#endif  // Cloud_Fusion_H

