/** ROS node 

/* 

Copyright (c) 2017

*/

#pragma once

#include "icp_align_tool.h"

namespace Multi_Sensor_Alignment
{
  class Cloud_Alignment_Double : public Cloud_Alignment
  {
    
  public:
    Cloud_Alignment_Double(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle, int buffer_size);

    /** Destructor */
    ~Cloud_Alignment_Double();

    //Methods
    bool pushTransform();

    //service callbacks
    bool pushtransform_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);

  }; // class Cloud_Alignment_Double
} // namespace Multi_Sensor_Alignment
