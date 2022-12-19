/** ROS node 

/* 

Copyright (c) 2017

*/

#ifndef SOURCE_CAMERA_H
#define SOURCE_CAMERA_H

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

namespace ERDC
{
  class SourceCamera
  {
    public:
      SourceCamera(ros::NodeHandle &_node_handle, const std::string &_image_topic, const std::string &_info_topic)
        : it_(_node_handle)

      {
        SourceCamera(_node_handle, _image_topic, _info_topic, true, "rgba", sensor_msgs::PointField::FLOAT32, "", "", "", 0, 0, false);
      }

      SourceCamera(ros::NodeHandle &_node_handle, const std::string &_image_in_topic, const std::string &_info_in_topic, const bool &_rectified, const std::string &_fieldLabel, const int &_fieldType, const std::string &_image_out_topic, const std::string &_info_out_topic, const std::string &_depth_out_topic, const int &_output_width, const int &_output_height, const bool &_rectify)
        : it_(_node_handle)
      {
        node_handle = _node_handle;
        mutex = new std::mutex();
        cameraImage_topic = _image_in_topic;
        cameraInfo_topic = _info_in_topic;
        cameraRectified = _rectified;
        field.name = _fieldLabel;
        field.count = 1;
        field.datatype = _fieldType;

        outputImage_topic = _image_out_topic;
        outputInfo_topic = _info_out_topic;
        outputDepth_topic = _depth_out_topic;
        outputWidth = _output_width;
        outputHeight = _output_height;
        outputRectify = _rectify;

        frameID = ""; 

        cameraImage_sub = new message_filters::Subscriber<sensor_msgs::Image>(node_handle, cameraImage_topic, 1);
        cameraInfo_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(node_handle, cameraInfo_topic, 1);
        cameraSync  = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo>(*cameraImage_sub, *cameraInfo_sub, 1);
  
        if(outputImage_topic != "") outputImage_pub = it_.advertiseCamera(outputImage_topic, 1);
        if(outputInfo_topic != "") outputInfo_pub = node_handle.advertise<sensor_msgs::CameraInfo>(outputInfo_topic,1);
        if(outputDepth_topic != "") outputDepth_pub = node_handle.advertise<sensor_msgs::Image>(outputDepth_topic,1);

        distortion_model = "plumb_bob";
        distortion_coefficients = cv::Mat(1, 5, CV_64F);
        intrinsic_matrix = cv::Mat(3, 3, CV_64F);
        rectification_matrix = cv::Mat(3, 3, CV_64F);
        projection_matrix = cv::Mat(3, 4, CV_64F);

        fx = 0;
        fy = 0;
        cx = 0;
        cy = 0;
      }

      SourceCamera(const SourceCamera& _oldCamera)
        : it_(_oldCamera.node_handle)
      {
        mutex = _oldCamera.mutex;
        node_handle = _oldCamera.node_handle;
        cameraImage_topic = _oldCamera.cameraImage_topic;
        cameraInfo_topic = _oldCamera.cameraInfo_topic;
        cameraRectified = _oldCamera.cameraRectified;
        field = sensor_msgs::PointField(_oldCamera.field);

        outputImage_topic = _oldCamera.outputImage_topic;
        outputInfo_topic = _oldCamera.outputInfo_topic;
        outputDepth_topic = _oldCamera.outputImage_topic;
        outputWidth = _oldCamera.outputWidth;
        outputHeight = _oldCamera.outputHeight;
        outputRectify = _oldCamera.outputRectify;

        frameID = _oldCamera.frameID;
        transform = _oldCamera.transform;

        cameraImage_sub = _oldCamera.cameraImage_sub;
        cameraInfo_sub = _oldCamera.cameraInfo_sub;
        cameraSync  = _oldCamera.cameraSync;
  
        if(outputImage_topic != "") outputImage_pub = _oldCamera.outputImage_pub;
        if(outputInfo_topic != "") outputInfo_pub = _oldCamera.outputInfo_pub;
        if(outputDepth_topic != "") outputDepth_pub = _oldCamera.outputDepth_pub;

        distortion_model = _oldCamera.distortion_model;
        distortion_coefficients = _oldCamera.distortion_coefficients.clone();
        intrinsic_matrix = _oldCamera.intrinsic_matrix.clone();
        rectification_matrix = _oldCamera.rectification_matrix.clone();
        projection_matrix = _oldCamera.projection_matrix.clone();

        image.header = _oldCamera.image.header;
        image.encoding = _oldCamera.image.encoding;
        image.image = _oldCamera.image.image.clone();

        fx = _oldCamera.fx;
        fy = _oldCamera.fy;
        cx = _oldCamera.cx;
        cy = _oldCamera.cy;

        camera_info_received = _oldCamera.camera_info_received;
      }

      ~SourceCamera() {}

      std::mutex* mutex;
      ros::NodeHandle node_handle;
      std::string cameraImage_topic, cameraInfo_topic;
      std::string outputImage_topic, outputInfo_topic, outputDepth_topic; 
      sensor_msgs::PointField field;

      message_filters::Subscriber<sensor_msgs::Image>* cameraImage_sub;
      message_filters::Subscriber<sensor_msgs::CameraInfo>* cameraInfo_sub;
      message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> *cameraSync;

      ros::Publisher outputInfo_pub;
      ros::Publisher outputDepth_pub;

      image_transport::ImageTransport it_;
      image_transport::CameraPublisher outputImage_pub;
      
      bool cameraRectified, outputRectify;
      uint32_t outputWidth, outputHeight;
      std::string distortion_model;
      cv::Mat distortion_coefficients;
      cv::Mat intrinsic_matrix;
      cv::Mat rectification_matrix;
      cv::Mat projection_matrix;
      cv_bridge::CvImage image;
 
      std::string frameID;
      geometry_msgs::TransformStamped transform;

      float fx;
      float fy;
      float cx;
      float cy;

      bool camera_info_received;
  };
  
} // namespace Multi_Sensor_Alignment

#endif  // SOURCE_CAMERA_H

