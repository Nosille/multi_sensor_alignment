
#include "icp_align_tool/icp_align_tool_single.h"

namespace Multi_Sensor_Alignment
{
  Cloud_Alignment_Single::Cloud_Alignment_Single(const ros::NodeHandle &node_handle, 
                      const ros::NodeHandle &private_node_handle, int buffer_size) 
  : Cloud_Alignment(node_handle, private_node_handle, buffer_size)
  {
    pnh_.param<std::string>("base_link_frame", base_link_frame_id_, "");

    pnh_.param<std::string>("fixed_frame", fixed_frame_id_, "");

    fixed_sensor_frame_id_ = parent_frame_id_ + "_fixed";
    child_frame_id_ = parent_frame_id_;

    initSensorFixedFrame();

    if (fixed_frame_id_ != ""){
      original_parent_frame_id_ = parent_frame_id_;
      parent_frame_id_ = fixed_sensor_frame_id_;
      child_frame_id_ = fixed_sensor_frame_id_;
        
      ROS_INFO_STREAM_NAMED(node_name, "original_parent_frame set to " << original_parent_frame_id_);
    }
    else {
      ROS_ERROR_STREAM_NAMED(node_name, "Set a `fixed_frame` if `input_cloud0 == input_cloud1`.");
      exit;
    }

    ROS_INFO_STREAM_NAMED(node_name, "fixed_frame set to " << fixed_frame_id_);
    ROS_INFO_STREAM_NAMED(node_name, "fixed_sensor_frame set to " << fixed_sensor_frame_id_);
    ROS_INFO_STREAM_NAMED(node_name, "base_link_frame set to " << base_link_frame_id_);
    ROS_INFO_STREAM_NAMED(node_name, "parent_frame set to " << parent_frame_id_);
    ROS_INFO_STREAM_NAMED(node_name, "child_frame set to " << child_frame_id_);

    //ROS services
    service7_ = pnh_.advertiseService("push_yaw", &Cloud_Alignment_Single::pushYaw_callback, this);
    service8_ = pnh_.advertiseService("push_roll_pitch_correction", &Cloud_Alignment_Single::pushRollPitchCorrection_callback, this);
  }

  Cloud_Alignment_Single::~Cloud_Alignment_Single()
  {

  }

  void Cloud_Alignment_Single::initSensorFixedFrame()
  {
    static tf2_ros::StaticTransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped baseLinkOrientationTransform;

    sensorFixedFrameTransform_ = tfBuffer_.lookupTransform(fixed_frame_id_, parent_frame_id_, ros::Time(0));

    sensorFixedFrameTransform_.header.frame_id = fixed_frame_id_;
    sensorFixedFrameTransform_.child_frame_id = fixed_sensor_frame_id_;

    // Clear Orientation, we only want to capture the translation
    sensorFixedFrameTransform_.transform.rotation.x = 0;
    sensorFixedFrameTransform_.transform.rotation.y = 0;
    sensorFixedFrameTransform_.transform.rotation.z = 0;
    sensorFixedFrameTransform_.transform.rotation.w = 1;

    // If base_link_frame is provided, we should use it's orientation instead
    if (base_link_frame_id_ != "" && fixed_frame_id_ != "") {
      baseLinkOrientationTransform = tfBuffer_.lookupTransform(fixed_frame_id_, base_link_frame_id_, ros::Time(0));

      sensorFixedFrameTransform_.transform.rotation = baseLinkOrientationTransform.transform.rotation;
    }

    broadcaster.sendTransform(sensorFixedFrameTransform_);
  }

    /**
   * @brief calculateYaw
   * ICP_y / (TF_x - ICP_x) = yawRotRAD
   * 
   * @return true 
   * @return false 
   */
  bool Cloud_Alignment_Single::calculateYaw()
  {
    geometry_msgs::TransformStamped captureMotion;
    ros::Time stamp = stamp_;
    
    captureMotion = tfBuffer_.lookupTransform(fixed_sensor_frame_id_, original_parent_frame_id_, stamp);

    xTF_ = captureMotion.transform.translation.x;
    yTF_ = captureMotion.transform.translation.y;
    zTF_ = captureMotion.transform.translation.z;

    double yICP = output_->transform.translation.y;
    double xICP = output_->transform.translation.x;

    current_yaw_ = (yTF_ - yICP) / (xTF_ - xICP);

    return true;
  }

  /**
   * @brief pushYaw
   * 
   * @return true 
   * @return false 
   */
  bool Cloud_Alignment_Single::pushYaw()
  {
    if(!received_alignPubConfig_)
    {
      // ROS_WARN_STREAM_NAMED(node_name, "Alignment Server isn't connected.");
      ROS_INFO_STREAM_NAMED(node_name, "Alignment Server isn't connected.");
      return true;
    }
    
    alignPubConfig_.yaw = current_yaw_;

    return alignClient_->setConfiguration(alignPubConfig_);
  }

  bool Cloud_Alignment_Single::calculateRollPitchCorrection() {
    tf2::Quaternion q;
    convert(output_->transform.rotation, q);
    tf2::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    current_roll_  = alignPubConfig_.roll - (roll / 2);
    current_pitch_ = alignPubConfig_.pitch - (pitch / 2);

    return true;
  }
  
  bool Cloud_Alignment_Single::pushRollPitchCorrection()
  {
    if(!received_alignPubConfig_)
    {
      // ROS_WARN_STREAM_NAMED(node_name, "Alignment Server isn't connected.");
      ROS_INFO_STREAM_NAMED(node_name, "Alignment Server isn't connected.");
      return true;
    }

    alignPubConfig_.roll  = current_roll_;
    alignPubConfig_.pitch = current_pitch_;

    return alignClient_->setConfiguration(alignPubConfig_);
  }

  
  
  bool Cloud_Alignment_Single::pushYaw_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Cloud_Alignment_Single::pushYaw();
  }
  
  bool Cloud_Alignment_Single::pushRollPitchCorrection_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Cloud_Alignment_Single::pushRollPitchCorrection();
  }
}  // namespace Multi_Sensor_Alignment