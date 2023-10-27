
#include "icp_align_tool/icp_align_tool_double.h"

namespace Multi_Sensor_Alignment
{
  Cloud_Alignment_Double::Cloud_Alignment_Double(const ros::NodeHandle &node_handle, 
                      const ros::NodeHandle &private_node_handle, int buffer_size) 
  : Cloud_Alignment(node_handle, private_node_handle, buffer_size)
  {
    ROS_INFO_STREAM_NAMED(node_name, "parent_frame set to " << parent_frame_id_);
    ROS_INFO_STREAM_NAMED(node_name, "child_frame set to " << child_frame_id_);

    // ROS services
    service6_ = pnh_.advertiseService("push_transform", &Cloud_Alignment_Double::pushtransform_callback, this);
  }

  Cloud_Alignment_Double::~Cloud_Alignment_Double()
  {

  }

  bool Cloud_Alignment_Double::pushTransform()
  {
    if(!received_alignPubConfig_)
    {
      // ROS_WARN_STREAM_NAMED(node_name, "Alignment Server isn't connected.");
      ROS_INFO_STREAM_NAMED(node_name, "Alignment Server isn't connected.");
      return true;
    }

    alignPubConfig_.x = output_->transform.translation.x;
    alignPubConfig_.y = output_->transform.translation.y;
    alignPubConfig_.z = output_->transform.translation.z;

    tf2::Quaternion q;
    convert(output_->transform.rotation, q);
    tf2::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    alignPubConfig_.roll = roll;
    alignPubConfig_.pitch = pitch;
    alignPubConfig_.yaw = yaw;

    return alignClient_->setConfiguration(alignPubConfig_);
  }

  bool Cloud_Alignment_Double::pushtransform_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Cloud_Alignment_Double::pushTransform();
  }
}  // namespace Multi_Sensor_Alignment





