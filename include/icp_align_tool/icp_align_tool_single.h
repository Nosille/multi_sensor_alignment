/** ROS node 

/* 

Copyright (c) 2017

*/

#pragma once

#include "icp_align_tool.h"

namespace Multi_Sensor_Alignment
{
  class Cloud_Alignment_Single : public Cloud_Alignment
  {
    
  public:
    Cloud_Alignment_Single(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle, int buffer_size);

    /** Destructor */
    ~Cloud_Alignment_Single();


    //Methods
    bool calculateYaw();
    bool pushYaw();
    bool calculateRollPitchCorrection();
    bool pushRollPitchCorrection();

    void initSensorFixedFrame();    

    //service callbacks
    bool pushYaw_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool pushRollPitchCorrection_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);

  protected:
    geometry_msgs::TransformStamped sensorFixedFrameTransform_;
    std::string fixed_frame_id_, fixed_sensor_frame_id_, original_parent_frame_id_;
    std::string base_link_frame_id_;
    
    double current_yaw_, current_pitch_, current_roll_;
    double xTF_, yTF_, zTF_;

  }; // class Cloud_Alignment_Single
} // namespace Multi_Sensor_Alignment
