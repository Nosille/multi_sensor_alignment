
#include "imu_tare_tool/imu_tare_tool.h"

namespace Multi_Sensor_Alignment
{
  IMU_Tare::IMU_Tare(const ros::NodeHandle &node_handle, 
                        const ros::NodeHandle &private_node_handle, int buffer_size)
  // Initialization list
  :nh_(node_handle),
  pnh_(private_node_handle),
  current_guess_(Eigen::Matrix4f::Identity()),
  output_(new geometry_msgs::TransformStamped),
  buffer_size_(buffer_size),
  qw_array_(tag::rolling_window::window_size = buffer_size),
  qx_array_(tag::rolling_window::window_size = buffer_size),
  qy_array_(tag::rolling_window::window_size = buffer_size),
  qz_array_(tag::rolling_window::window_size = buffer_size),
  current_qw_(0), current_qx_(0), current_qy_(0), current_qz_(0),
  tfListener_(tfBuffer_),
  wait_for_tf_delay_(0.1),
  received_alignPubConfig_(false),
  received_alignPubDesc_(false),
  received_tareToolConfig_(false)
  {
    this->onInit();
  }
  IMU_Tare::~IMU_Tare()
  {
    // pass
  }

  void IMU_Tare::onInit()
  {

    const std::string complete_ns = pnh_.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    node_name = complete_ns.substr(id + 1);

    tfBuffer_.setUsingDedicatedThread(true);

  //Setup Dynamic Reconfigure Server for imuTareToolConfig
    dynamic_reconfigure::Server<multi_sensor_alignment::imu_tare_toolConfig>::CallbackType
        drServerCallback_ = boost::bind(&IMU_Tare::reconfigure_server_callback, this, _1, _2);
    drServer_.reset(new dynamic_reconfigure::Server<multi_sensor_alignment::imu_tare_toolConfig>(drServer_mutex_, pnh_));
    drServer_->setCallback(drServerCallback_);

    //Wait on this nodes dyanamic param server to intialize values
    while(!received_tareToolConfig_)
    {
      ros::Duration(1.0).sleep();
      ROS_INFO_STREAM_NAMED(node_name, "Waiting on dynamic parameters.");
      ros::spinOnce();
    }

  //Subscribe to Dynamic Reconfigure on the alignment publisher node, AlignPubConfig
    pnh_.param<std::string>("alignment_server", align_server_name_, "");
      ROS_INFO_STREAM_NAMED(node_name, "alignment_server set to " << align_server_name_);   
    alignClient_.reset(new dynamic_reconfigure::Client<multi_sensor_alignment::alignment_publisherConfig>(align_server_name_));
    alignClient_->setConfigurationCallback(boost::bind(&IMU_Tare::align_pubconfig_callback, this, _1));
    alignClient_->setDescriptionCallback(boost::bind(&IMU_Tare::align_pubdesc_callback, this, _1));
    
    //Wait 60 seconds for the alignment publisher nodes dynamic param server to respond
    int count = 0, maxcount = 60;
    while((count < maxcount && (!received_alignPubConfig_ || !received_alignPubDesc_)) && align_server_name_ != "")
    {
      ros::Duration(1.0).sleep();
      ROS_INFO_STREAM_NAMED(node_name, "Waiting on dynamic parameters from align_publisher. " << (maxcount-count) << " sec before giving up.");
      ros::spinOnce();
      count++;
    }

  // ROS Parameters

    pnh_.param("output_frequency", output_frequency_, 10.0);
      ROS_INFO_STREAM_NAMED(node_name, "output_frequency set to " << output_frequency_);     
          

    if(!received_alignPubConfig_)
    {
      ROS_INFO_STREAM_NAMED(node_name, "Proceeding without alignment publisher.");
    }

    ROS_INFO_STREAM_NAMED(node_name, "x set to " << alignPubConfig_.x);
    ROS_INFO_STREAM_NAMED(node_name, "y set to " << alignPubConfig_.y);
    ROS_INFO_STREAM_NAMED(node_name, "z set to " << alignPubConfig_.z);
    ROS_INFO_STREAM_NAMED(node_name, "roll set to " << alignPubConfig_.roll);
    ROS_INFO_STREAM_NAMED(node_name, "pitch set to " << alignPubConfig_.pitch);
    ROS_INFO_STREAM_NAMED(node_name, "yaw set to " << alignPubConfig_.yaw);

    pnh_.param<std::string>("input_topic", input_topic_, "imu");
      ROS_INFO_STREAM_NAMED(node_name, "input_topic set to " << input_topic_);
    pnh_.param<std::string>("output", output_trans_topic_, "output");
      ROS_INFO_STREAM_NAMED(node_name, "output topic set to " << output_trans_topic_);

    // pnh_.param<std::string>("fixed_frame", fixed_frame_id_, "");
    pnh_.param<std::string>("base_link_frame", base_link_frame_id_, "");
    pnh_.param<std::string>("parent_frame", parent_frame_id_, "");
    pnh_.param<std::string>("child_frame", child_frame_id_, "");

    // ROS_INFO_STREAM_NAMED(node_name, "fixed_frame set to " << fixed_frame_id_);
    ROS_INFO_STREAM_NAMED(node_name, "base_link_frame set to " << base_link_frame_id_);
    ROS_INFO_STREAM_NAMED(node_name, "parent_frame set to " << parent_frame_id_);
    ROS_INFO_STREAM_NAMED(node_name, "child_frame set to " << child_frame_id_);
      
    drServer_->updateConfig(tareToolConfig_);

  // ROS publishers
    output_trans_pub_  = nh_.advertise<geometry_msgs::TransformStamped>(output_trans_topic_,100);
    pub_timer_ = nh_.createTimer(ros::Duration(1.0/output_frequency_), boost::bind(& IMU_Tare::publish_callback, this, _1));

  // ROS subscribers
    input_sub_ = nh_.subscribe(input_topic_, 100, &IMU_Tare::input_callback, this);

  // ROS Services
    service0_ = pnh_.advertiseService("capture", &IMU_Tare::capture_callback, this);

    if(received_alignPubConfig_)
    {
      service1_ = pnh_.advertiseService("push_yaw_correction", &IMU_Tare::pushYawCorrection_callback, this);
      service2_ = pnh_.advertiseService("push_roll_pitch_correction", &IMU_Tare::pushRollPitchCorrection_callback, this);
      service3_ = pnh_.advertiseService("revert", &IMU_Tare::revert_callback, this);
      service4_ = pnh_.advertiseService("reset", &IMU_Tare::reset_callback, this);
    } 
    else {
      ROS_WARN_STREAM_NAMED(node_name, "IMU_TARE_TOOL did not find the alignment publisher.");
    }

  // Reset the guess transform for good measure
    IMU_Tare::revert();

    ROS_INFO_STREAM_NAMED(node_name, node_name.c_str() << " initialized!");
  }

  void IMU_Tare::reconfigure_server_callback(multi_sensor_alignment::imu_tare_toolConfig &config, uint32_t level) 
  {
    
    ROS_INFO("Reconfigure Request: ");

    tareToolConfig_ = config;
    received_tareToolConfig_ = true;

    IMU_Tare::revert();
  }

  void IMU_Tare::align_pubconfig_callback(const multi_sensor_alignment::alignment_publisherConfig& config) 
  {
    ROS_INFO("Received configuration from alignment publisher");
    ROS_INFO("%f %f %f %f %f %f", 
            config.x, config.y, config.z, config.roll, config.pitch, config.yaw);

    if (!received_alignPubConfig_) 
      initialAlignPubConfig_ = config;

    alignPubConfig_ = config;
    received_alignPubConfig_ = true;

    IMU_Tare::revert();
  }

  void IMU_Tare::align_pubdesc_callback(const dynamic_reconfigure::ConfigDescription& description) 
  {
    ROS_INFO("Received description from alignment publisher");

    alignPubDesc_ = description;
    received_alignPubDesc_ = true;
  }

   /**
   * @brief pushYaw
   * 
   * @return true 
   * @return false 
   */
  bool IMU_Tare::pushYawCorrection()
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

  bool IMU_Tare::pushRollPitchCorrection()
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
  
  void IMU_Tare::publish_callback(const ros::TimerEvent& event)
  {
    ROS_INFO_STREAM_NAMED(node_name, "---");

  // //convert transformation to ros usable form
  //   Eigen::Matrix4f mf = (current_guess_);
  //   Eigen::Matrix4d md(mf.cast<double>());
  //   Eigen::Affine3d affine(md);
  //   geometry_msgs::TransformStamped transformStamped = tf2::eigenToTransform(affine);
  //   output_->transform = transformStamped.transform;
  //   output_->header = imu_msg_.header;
  //   output_->header.frame_id = parent_frame;
  //   output_->child_frame_id = child_frame;

  //   output_->header = sensor_msgs::Imu:fromPCL(cloud1->header);

    if (!capture_) return;

    output_->transform.translation.x = 0;
    output_->transform.translation.y = 0;
    output_->transform.translation.z = 0;
    output_->transform.rotation.x = bs_to_world_transform_.transform.rotation.x;
    output_->transform.rotation.y = bs_to_world_transform_.transform.rotation.y;
    output_->transform.rotation.z = bs_to_world_transform_.transform.rotation.z;
    output_->transform.rotation.w = bs_to_world_transform_.transform.rotation.w;
    
    //Add new transform to accumulator and compute average
    if(buffer_size_ > 1)
    {
      geometry_msgs::Quaternion q_mean = IMU_Tare::AverageQuaternion(output_->transform.rotation);

      //Update transformations with average
      output_->transform.translation.x = 0;
      output_->transform.translation.y = 0;
      output_->transform.translation.z = 0;
      output_->transform.rotation = q_mean;

      Eigen::Affine3d eigenTransform = tf2::transformToEigen(output_->transform);
      current_guess_ = eigenTransform.matrix().cast<float>();
    }
    
    // Calculate diff from last_transform
    tf2::Transform old_trans, new_trans;
    tf2::convert(last_transform_, old_trans);
    tf2::convert(output_->transform, new_trans);
    tf2::Transform diff = old_trans.inverseTimes(new_trans);
    
    geometry_msgs::TransformStamped gm_diff;
    tf2::convert(diff, gm_diff.transform);
    gm_diff.header = bs_to_world_transform_.header;
    // gm_diff.header.frame_id = child_frame;
    // gm_diff.child_frame_id = child_frame;

    tf2::Matrix3x3 diff_matrix(diff.getRotation());
    double diff_roll, diff_pitch, diff_yaw;
    diff_matrix.getRPY(diff_roll, diff_pitch, diff_yaw);

    //writeout values
    double roll,pitch,yaw;

    tf2::Quaternion q(
          output_->transform.rotation.x,
          output_->transform.rotation.y,
          output_->transform.rotation.z,
          output_->transform.rotation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll,pitch,yaw);

    ROS_INFO_STREAM_NAMED(node_name, "Roll:  " << roll <<  " rad, " << (roll/PI*180)  << " deg" << ", diff: " << diff_roll << " rad");
    ROS_INFO_STREAM_NAMED(node_name, "pitch: " << pitch << " rad, " << (pitch/PI*180) << " deg" << ", diff: " << diff_pitch << " rad");
    ROS_INFO_STREAM_NAMED(node_name, "Yaw:   " << yaw <<   " rad, " << (yaw/PI*180)   << " deg" << ", diff: " << diff_yaw << " rad");

    // Create output msgs
    output_trans_pub_.publish(output_);
  }

  void IMU_Tare::input_callback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    geometry_msgs::TransformStamped imu_transform; // transform from imu frame to world frame
    geometry_msgs::TransformStamped bs_transform; // transform from baselink frame to world frame

    imu_transform.transform.translation.x = 0;
    imu_transform.transform.translation.y = 0;
    imu_transform.transform.translation.z = 0;
    imu_transform.transform.rotation.x = msg->orientation.x;
    imu_transform.transform.rotation.y = msg->orientation.y;
    imu_transform.transform.rotation.z = msg->orientation.z;
    imu_transform.transform.rotation.w = msg->orientation.w;

    geometry_msgs::TransformStamped transform;
  ;

    if(base_link_frame_id_ != "")
    {
      try{
        transform = tfBuffer_.lookupTransform(base_link_frame_id_, msg->header.frame_id, msg->header.stamp);
        
        tf2::doTransform(imu_transform, bs_transform, transform);

        bs_to_world_transform_ = bs_transform;
      }
      catch (tf2::TransformException ex){
        ROS_WARN("%s",ex.what());
        return;
      }
    }

  }

  bool IMU_Tare::capture_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    capture_ = true;

    return true;	  
  }

  bool IMU_Tare::reset()
  {
    alignPubConfig_ = initialAlignPubConfig_;
    return alignClient_->setConfiguration(alignPubConfig_);
  }
  
  bool IMU_Tare::revert()
  {
    //revert Guess
    last_transform_ = tfBuffer_.lookupTransform(parent_frame_id_, child_frame_id_, ros::Time::now()).transform;
    
    Eigen::Affine3d eigenTransform = tf2::transformToEigen(last_transform_);
    current_guess_ = eigenTransform.matrix().cast<float>();
    
    //revert rolling window accumulators
    qx_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qy_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qz_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qw_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    current_qx_ = 0; current_qy_ = 0; current_qz_ = 0; current_qw_ = 0; 
    
    ROS_INFO_STREAM_NAMED(node_name, "Guess transform reverted");

    return true;
  }

  bool IMU_Tare::revert_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return IMU_Tare::revert();
  }

  bool IMU_Tare::reset_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return IMU_Tare::reset();
  }

  bool IMU_Tare::pushYawCorrection_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return IMU_Tare::pushYawCorrection();
  }
  
  bool IMU_Tare::pushRollPitchCorrection_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return IMU_Tare::pushRollPitchCorrection();
  }
  

  geometry_msgs::Quaternion IMU_Tare::AverageQuaternion(const geometry_msgs::Quaternion& newRotation)
  {
    //ROS_INFO_STREAM_NAMED(node_name, current_qx_ << " " << current_qy_ << " " << current_qz_ << " " << current_qw_);
    tf2::Quaternion lastRotation(current_qx_, current_qy_, current_qz_, current_qw_);
    tf2::Quaternion currRotation; 
    tf2::convert(newRotation, currRotation);
    //On first pass lastRotation will be zero length
    if(abs(lastRotation.length()) < 0.1)
    {
      ROS_INFO_STREAM_NAMED(node_name, "AverageQuaternion initialized");

      //Add new values to accumulators
      qx_array_(currRotation.x()); current_qx_ = currRotation.x();
      qy_array_(currRotation.y()); current_qy_ = currRotation.y();
      qz_array_(currRotation.z()); current_qz_ = currRotation.z();
      qw_array_(currRotation.w()); current_qw_ = currRotation.w();

      return newRotation;
    }

    //Before we add the new rotation to the average (mean), we have to check whether the quaternion has to be inverted. Because
    //q and -q are the same rotation, but cannot be averaged, we have to make sure they are all the same.
    // if(AreQuaternionsClose(currRotation, lastRotation))
    // {
    //     ROS_INFO_STREAM_NAMED(node_name, "flip quaternion");
    //     ROS_INFO_STREAM_NAMED(node_name, currRotation.x() << " " << currRotation.y() << " " << currRotation.z() << " " << currRotation.w());
    //     ROS_INFO_STREAM_NAMED(node_name, lastRotation.x() << " " << lastRotation.y() << " " << lastRotation.z() << " " << lastRotation.w());
    //     currRotation = tf2::Quaternion(-currRotation.x(), -currRotation.y(), -currRotation.z(), -currRotation.w());
    // }
    current_qx_ = currRotation.x();
    current_qy_ = currRotation.y();
    current_qz_ = currRotation.z();
    current_qw_ = currRotation.w();
    
    //Add new values to accumulators
    qx_array_(currRotation.x());
    qy_array_(currRotation.y());
    qz_array_(currRotation.z());
    qw_array_(currRotation.w());
    float w = rolling_mean(qw_array_);
    float x = rolling_mean(qx_array_);
    float y = rolling_mean(qy_array_);
    float z = rolling_mean(qz_array_);

    //Convert back to quaternion
    tf2::Quaternion mean(x, y, z, w);

    geometry_msgs::Quaternion result;
    tf2::convert(mean.normalize(), result);

    //note: if speed is an issue, you can skip the normalization step
    return result;
}

//Returns true if the two input quaternions are close to each other. This can
//be used to check whether or not one of two quaternions which are supposed to
//be very similar but has its component signs reversed (q has the same rotation as
//-q)
bool IMU_Tare::AreQuaternionsClose(tf2::Quaternion q1, tf2::Quaternion q2)
{

    float dot = q1.dot(q2);
    
    if(dot < 0.0f)
    {

        return false;                   
    }

    else
    {

        return true;
    }
}

  
}  // namespace Multi_Sensor_Alignment





