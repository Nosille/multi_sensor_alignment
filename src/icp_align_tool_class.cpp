
#include "icp_align_tool/icp_align_tool.h"

namespace Multi_Sensor_Alignment
{
  Cloud_Alignment::Cloud_Alignment(const ros::NodeHandle &node_handle, 
                        const ros::NodeHandle &private_node_handle, int buffer_size)
  // Initialization list
  :nh_(node_handle),
  pnh_(private_node_handle),
  freeze0_(0),
  freeze1_(0),
  current_guess_(Eigen::Matrix4f::Identity()),
  output_transform_(new geometry_msgs::TransformStamped),
  buffer_size_(buffer_size),
  x_array_(tag::rolling_window::window_size = buffer_size),
  y_array_(tag::rolling_window::window_size = buffer_size),
  z_array_(tag::rolling_window::window_size = buffer_size),
  qw_array_(tag::rolling_window::window_size = buffer_size),
  qx_array_(tag::rolling_window::window_size = buffer_size),
  qy_array_(tag::rolling_window::window_size = buffer_size),
  qz_array_(tag::rolling_window::window_size = buffer_size),
  current_qw_(0), current_qx_(0), current_qy_(0), current_qz_(0),
  tfListener_(tfBuffer_),
  wait_for_tf_delay_(0.1),
  received_alignPubConfig_(false),
  received_alignPubDesc_(false),
  received_alignToolConfig_(false)
  {
    this->onInit();
  }
  Cloud_Alignment::~Cloud_Alignment()
  {
    // pass
  }

  void Cloud_Alignment::onInit()
  {

    const std::string complete_ns = pnh_.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    node_name = complete_ns.substr(id + 1);

    tfBuffer_.setUsingDedicatedThread(true);

  //Setup Dynamic Reconfigure Server for alignCheckConfig
    dynamic_reconfigure::Server<multi_sensor_alignment::icp_align_toolConfig>::CallbackType
        drServerCallback_ = boost::bind(&Cloud_Alignment::reconfigure_server_callback, this, _1, _2);
    drServer_.reset(new dynamic_reconfigure::Server<multi_sensor_alignment::icp_align_toolConfig>(drServer_mutex_, pnh_));
    drServer_->setCallback(drServerCallback_);

    //Wait on this nodes dyanamic param server to intialize values
    while(!received_alignToolConfig_)
    {
      ros::Duration(1.0).sleep();
      ROS_INFO_STREAM_NAMED(node_name, "Waiting on dynamic parameters.");
      ros::spinOnce();
    }

  //Subscribe to Dynamic Reconfigure on the alignment publisher node, AlignPubConfig
    pnh_.param<std::string>("alignment_server", align_server_name_, "");
      ROS_INFO_STREAM_NAMED(node_name, "alignment_server set to " << align_server_name_);   
    alignClient_.reset(new dynamic_reconfigure::Client<multi_sensor_alignment::alignment_publisherConfig>(align_server_name_));
    alignClient_->setConfigurationCallback(boost::bind(&Cloud_Alignment::align_pubconfig_callback, this, _1));
    alignClient_->setDescriptionCallback(boost::bind(&Cloud_Alignment::align_pubdesc_callback, this, _1));
    
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

    pnh_.param<std::string>("input_cloud0", input0_topic_, "input0");
      ROS_INFO_STREAM_NAMED(node_name, "input_cloud0 topic set to " << input0_topic_);
    pnh_.param<std::string>("input_cloud1", input1_topic_, "input1");
      ROS_INFO_STREAM_NAMED(node_name, "input_cloud1 topic set to " << input1_topic_);
    pnh_.param<std::string>("output_cloud0", output_cloud0_topic_, "cloud0");
      ROS_INFO_STREAM_NAMED(node_name, "output_cloud0 topic set to " << output_cloud0_topic_);
    pnh_.param<std::string>("output_cloud1", output_cloud1_topic_, "cloud1");
      ROS_INFO_STREAM_NAMED(node_name, "output_cloud1 topic set to " << output_cloud1_topic_);
    pnh_.param<std::string>("output", output_trans_topic_, "output");
      ROS_INFO_STREAM_NAMED(node_name, "output topic set to " << output_trans_topic_);
    pnh_.param("is_output_filtered", is_output_filtered_, false);
      ROS_INFO_STREAM_NAMED(node_name, "is_output_filtered set to " << is_output_filtered_);

    pnh_.param<std::string>("fixed_frame", fixed_frame_id_, "");
    pnh_.param<std::string>("base_link_frame", base_link_frame_id_, "");
    pnh_.param<std::string>("parent_frame", parent_frame_id_, "");
    pnh_.param<std::string>("child_frame", child_frame_id_, "");

    fixed_sensor_frame_id_ = "";

    if (input0_topic_ == input1_topic_) {
      lidar_to_robot_ = true;

      fixed_sensor_frame_id_ = parent_frame_id_ + "_fixed";
      
      initSensorFixedFrame();

      if (child_frame_id_ != parent_frame_id_ && fixed_frame_id_ != ""){
        original_parent_frame_id_ = parent_frame_id_;
        parent_frame_id_ = fixed_sensor_frame_id_;
        child_frame_id_ = fixed_sensor_frame_id_;
        
        ROS_INFO_STREAM_NAMED(node_name, "original_parent_frame set to " << original_parent_frame_id_);
      }
      else {
        ROS_ERROR_STREAM_NAMED(node_name, "Set a `fixed_frame` if `input_cloud0 == input_cloud1`.");
        exit;
      }
    }
    else {
      lidar_to_robot_ = false;
    }

    ROS_INFO_STREAM_NAMED(node_name, "Lidar to Robot Mode: " << lidar_to_robot_);

    ROS_INFO_STREAM_NAMED(node_name, "fixed_frame set to " << fixed_frame_id_);
    ROS_INFO_STREAM_NAMED(node_name, "fixed_sensor_frame set to " << fixed_sensor_frame_id_);
    ROS_INFO_STREAM_NAMED(node_name, "base_link_frame set to " << base_link_frame_id_);
    ROS_INFO_STREAM_NAMED(node_name, "parent_frame set to " << parent_frame_id_);
    ROS_INFO_STREAM_NAMED(node_name, "child_frame set to " << child_frame_id_);

      
    pnh_.param("voxelSize", alignToolConfig_.VoxelSize, alignToolConfig_.VoxelSize);
      ROS_INFO_STREAM_NAMED(node_name, "voxelSize set to " << alignToolConfig_.VoxelSize);

    pnh_.param("filter/i_min", alignToolConfig_.i_min, alignToolConfig_.i_min);
      ROS_INFO_STREAM_NAMED(node_name, "filter min i set to " << alignToolConfig_.i_min);   
    pnh_.param("filter/i_max", alignToolConfig_.i_max, alignToolConfig_.i_max);
      ROS_INFO_STREAM_NAMED(node_name, "filter max i set to " << alignToolConfig_.i_max);  
    pnh_.param("filter/x_min", alignToolConfig_.x_min, alignToolConfig_.x_min);
      ROS_INFO_STREAM_NAMED(node_name, "filter min x set to " << alignToolConfig_.x_min);   
    pnh_.param("filter/x_max", alignToolConfig_.x_max, alignToolConfig_.x_max);
      ROS_INFO_STREAM_NAMED(node_name, "filter max x set to " << alignToolConfig_.x_max);   
    pnh_.param("filter/y_min", alignToolConfig_.y_min, alignToolConfig_.y_min);
      ROS_INFO_STREAM_NAMED(node_name, "filter min y set to " << alignToolConfig_.y_min);   
    pnh_.param("filter/y_max", alignToolConfig_.y_max, alignToolConfig_.y_max);
      ROS_INFO_STREAM_NAMED(node_name, "filter max y set to " << alignToolConfig_.y_max);   
    pnh_.param("filter/z_min", alignToolConfig_.z_min, alignToolConfig_.z_min);
      ROS_INFO_STREAM_NAMED(node_name, "filter min z set to " << alignToolConfig_.z_min);   
    pnh_.param("filter/z_max", alignToolConfig_.z_max, alignToolConfig_.z_max);
      ROS_INFO_STREAM_NAMED(node_name, "filter max z set to " << alignToolConfig_.z_max);

    pnh_.param("method", alignToolConfig_.Method, 1);
      ROS_INFO_STREAM_NAMED(node_name, "method set to " << alignToolConfig_.Method);
    pnh_.param("epsilon", alignToolConfig_.Epsilon, alignToolConfig_.Epsilon);
      ROS_INFO_STREAM_NAMED(node_name, "epsilon set to " << alignToolConfig_.Epsilon);
    pnh_.param("maxIterations", alignToolConfig_.MaxIterations, alignToolConfig_.MaxIterations);
      ROS_INFO_STREAM_NAMED(node_name, "maxIterations set to " << alignToolConfig_.MaxIterations);
    pnh_.param("maxCorrespondenceDistance", alignToolConfig_.MaxCorrespondenceDistance, alignToolConfig_.MaxCorrespondenceDistance);
      ROS_INFO_STREAM_NAMED(node_name, "maxCorrespondenceDistance set to " << alignToolConfig_.MaxCorrespondenceDistance);

    pnh_.param("norm/KSearch", alignToolConfig_.KSearch, alignToolConfig_.KSearch);
      ROS_INFO_STREAM_NAMED(node_name, "KSearch set to " << alignToolConfig_.KSearch);
    pnh_.param("norm/RadiusSearch", alignToolConfig_.RadiusSearch, alignToolConfig_.RadiusSearch);
      ROS_INFO_STREAM_NAMED(node_name, "RadiusSearch set to " << alignToolConfig_.RadiusSearch);      
    
    pnh_.param("ndt/StepSize", alignToolConfig_.StepSize, alignToolConfig_.StepSize);
      ROS_INFO_STREAM_NAMED(node_name, "StepSize set to " << alignToolConfig_.StepSize);     
    pnh_.param("ndt/Resolution", alignToolConfig_.Resolution, alignToolConfig_.Resolution);
      ROS_INFO_STREAM_NAMED(node_name, "Resolution set to " << alignToolConfig_.Resolution);   
    
    drServer_->updateConfig(alignToolConfig_);

  // ROS publishers
    output_trans_pub_  = nh_.advertise<geometry_msgs::TransformStamped>(output_trans_topic_,100);
    output_cloud0_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud0_topic_,10);
    output_cloud1_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud1_topic_,10);
    pub_timer_ = nh_.createTimer(ros::Duration(1.0/output_frequency_), boost::bind(& Cloud_Alignment::publish_callback, this, _1));

  // ROS subscribers
    input_sub0_ = nh_.subscribe(input0_topic_, 100, &Cloud_Alignment::input0_callback, this);
    input_sub1_ = nh_.subscribe(input1_topic_, 100, &Cloud_Alignment::input1_callback, this);

  // ROS Services
    service0_ = pnh_.advertiseService("freeze_cloud0", &Cloud_Alignment::freeze0_callback, this);
    service1_ = pnh_.advertiseService("freeze_cloud1", &Cloud_Alignment::freeze1_callback, this);
    service2_ = pnh_.advertiseService("unfreeze_cloud0", &Cloud_Alignment::unfreeze0_callback, this);
    service3_ = pnh_.advertiseService("unfreeze_cloud1", &Cloud_Alignment::unfreeze1_callback, this);

    if(received_alignPubConfig_)
    {
      service4_ = pnh_.advertiseService("revert", &Cloud_Alignment::revert_callback, this);
      service5_ = pnh_.advertiseService("reset", &Cloud_Alignment::reset_callback, this);
      
      if(lidar_to_robot_) {
        service7_ = pnh_.advertiseService("push_yaw", &Cloud_Alignment::pushYaw_callback, this);
        service8_ = pnh_.advertiseService("push_roll_pitch_correction", &Cloud_Alignment::pushRollPitchCorrection_callback, this);
      } else {
        service6_ = pnh_.advertiseService("push_transform", &Cloud_Alignment::pushtransform_callback, this);
      }
    } 
    else {
      ROS_WARN_STREAM_NAMED(node_name, "ICP_ALIGN_TOOL did not find the alignment publisher.");
    }

  // Reset the guess transform for good measure
    Cloud_Alignment::revert();

    ROS_INFO_STREAM_NAMED(node_name, node_name.c_str() << " initialized!");
  }

  void Cloud_Alignment::initSensorFixedFrame()
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
    if (base_link_frame_id_ != "") {
      baseLinkOrientationTransform = tfBuffer_.lookupTransform(fixed_frame_id_, base_link_frame_id_, ros::Time(0));

      sensorFixedFrameTransform_.transform.rotation = baseLinkOrientationTransform.transform.rotation;
    }

    broadcaster.sendTransform(sensorFixedFrameTransform_);
  }

  void Cloud_Alignment::reconfigure_server_callback(multi_sensor_alignment::icp_align_toolConfig &config, uint32_t level) 
  {
    
    ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %f %f %d %f %d %f %d %f %f %f", 
            config.i_min,
            config.i_max, 
            config.x_min,
            config.x_max, 
            config.y_min,
            config.y_max, 
            config.z_min,
            config.z_max, 
            config.VoxelSize, 

            config.Method,
            config.Epsilon, 
            config.MaxIterations, 
            config.MaxCorrespondenceDistance,

            config.KSearch,
            config.RadiusSearch,
            
            config.StepSize,
            config.Resolution);

    alignToolConfig_ = config;
    received_alignToolConfig_ = true;

    Cloud_Alignment::revert();
  }

  void Cloud_Alignment::align_pubconfig_callback(const multi_sensor_alignment::alignment_publisherConfig& config) 
  {
    ROS_INFO("Received configuration from alignment publisher");
    ROS_INFO("%f %f %f %f %f %f", 
            config.x, config.y, config.z, config.roll, config.pitch, config.yaw);

    if (!received_alignPubConfig_) 
      initialAlignPubConfig_ = config;

    alignPubConfig_ = config;
    received_alignPubConfig_ = true;

    Cloud_Alignment::revert();
  }

  void Cloud_Alignment::align_pubdesc_callback(const dynamic_reconfigure::ConfigDescription& description) 
  {
    ROS_INFO("Received description from alignment publisher");

    alignPubDesc_ = description;
    received_alignPubDesc_ = true;
  }

  bool Cloud_Alignment::pushTransform()
  {
    if(!received_alignPubConfig_)
    {
      // ROS_WARN_STREAM_NAMED(node_name, "Alignment Server isn't connected.");
      ROS_INFO_STREAM_NAMED(node_name, "Alignment Server isn't connected.");
      return true;
    }

    alignPubConfig_.x = output_transform_->transform.translation.x;
    alignPubConfig_.y = output_transform_->transform.translation.y;
    alignPubConfig_.z = output_transform_->transform.translation.z;

    tf2::Quaternion q;
    convert(output_transform_->transform.rotation, q);
    tf2::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    alignPubConfig_.roll = roll;
    alignPubConfig_.pitch = pitch;
    alignPubConfig_.yaw = yaw;

    return alignClient_->setConfiguration(alignPubConfig_);
  }
  
  /**
   * @brief calculateYaw
   * ICP_y / (TF_x - ICP_x) = yawRotRAD
   * 
   * @return true 
   * @return false 
   */
  bool Cloud_Alignment::calculateYaw()
  {
    geometry_msgs::TransformStamped captureMotion;
    ros::Time stamp = stamp_;
    
    captureMotion = tfBuffer_.lookupTransform(fixed_sensor_frame_id_, original_parent_frame_id_, stamp);

    xTF_ = captureMotion.transform.translation.x;
    yTF_ = captureMotion.transform.translation.y;
    zTF_ = captureMotion.transform.translation.z;

    double yICP = output_transform_->transform.translation.y;
    double xICP = output_transform_->transform.translation.x;

    current_yaw_ = (yTF_ - yICP) / (xTF_ - xICP);

    return true;
  }

  /**
   * @brief pushYaw
   * 
   * @return true 
   * @return false 
   */
  bool Cloud_Alignment::pushYaw()
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

  bool Cloud_Alignment::calculateRollPitchCorrection() {
    tf2::Quaternion q;
    convert(output_transform_->transform.rotation, q);
    tf2::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    current_roll_  = alignPubConfig_.roll - (roll / 2);
    current_pitch_ = alignPubConfig_.pitch - (pitch / 2);

    return true;
  }
  
  bool Cloud_Alignment::pushRollPitchCorrection()
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
  
  void Cloud_Alignment::publish_callback(const ros::TimerEvent& event)
  {
    ROS_INFO_STREAM_NAMED(node_name, "---");

    if(cloud0_.data.size() <= 0 || cloud1_.data.size() <=0) return;

  // Convert from ROS msg to pointcloud2 object
    pcl::PointCloud<PointT>::Ptr cloud0(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(cloud0_, *cloud0);
    pcl::fromROSMsg(cloud1_, *cloud1);

    std::string parent_frame = cloud0_.header.frame_id;
    std::string child_frame  = cloud1_.header.frame_id;

  //Downsample
    pcl::PointCloud<PointT>::Ptr filtered_cloud0(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr filtered_cloud1(new pcl::PointCloud<PointT>);
    Multi_Sensor_Alignment::DownsampleCloud(cloud0, filtered_cloud0, alignToolConfig_.VoxelSize,
      alignToolConfig_.i_min, alignToolConfig_.i_max, 
      alignToolConfig_.x_min, alignToolConfig_.x_max,
      alignToolConfig_.y_min, alignToolConfig_.y_max,
      alignToolConfig_.z_min, alignToolConfig_.z_max);
    Multi_Sensor_Alignment::DownsampleCloud(cloud1, filtered_cloud1, alignToolConfig_.VoxelSize,
      alignToolConfig_.i_min, alignToolConfig_.i_max, 
      alignToolConfig_.x_min, alignToolConfig_.x_max,
      alignToolConfig_.y_min, alignToolConfig_.y_max,
      alignToolConfig_.z_min, alignToolConfig_.z_max);
    
    ROS_INFO_STREAM_NAMED(node_name, "\n");

  //Perform Registration
    Multi_Sensor_Alignment::PerformRegistration(filtered_cloud0, filtered_cloud1,
         current_guess_, alignToolConfig_.Method, alignToolConfig_.MaxIterations, alignToolConfig_.Epsilon,  
         alignToolConfig_.KSearch, alignToolConfig_.RadiusSearch, alignToolConfig_.MaxCorrespondenceDistance, 
         alignToolConfig_.StepSize, alignToolConfig_.Resolution);
    
  //convert transformation to ros usable form
    Eigen::Matrix4f mf = (current_guess_);
    Eigen::Matrix4d md(mf.cast<double>());
    Eigen::Affine3d affine(md);
    geometry_msgs::TransformStamped transformStamped = tf2::eigenToTransform(affine);
    output_transform_->transform = transformStamped.transform;
    output_transform_->header = pcl_conversions::fromPCL(cloud1->header);
    output_transform_->header.frame_id = parent_frame;
    output_transform_->child_frame_id = child_frame;

    //Add new transform to accumulator and compute average
    if(buffer_size_ > 1)
    {
      x_array_(output_transform_->transform.translation.x); double x_mean = rolling_mean(x_array_);
      y_array_(output_transform_->transform.translation.y); double y_mean = rolling_mean(y_array_);
      z_array_(output_transform_->transform.translation.z); double z_mean = rolling_mean(z_array_);
      geometry_msgs::Quaternion q_mean = Multi_Sensor_Alignment::AverageQuaternion(output_transform_->transform.rotation,
          qx_array_, qy_array_, qz_array_, qw_array_, current_qx_, current_qy_, current_qz_, current_qw_);

      //Update transformations with average
      output_transform_->transform.translation.x = x_mean;
      output_transform_->transform.translation.y = y_mean;
      output_transform_->transform.translation.z = z_mean;
      output_transform_->transform.rotation = q_mean;

      Eigen::Affine3d eigenTransform = tf2::transformToEigen(output_transform_->transform);
      current_guess_ = eigenTransform.matrix().cast<float>();
    }

    // Calculate diff from last_transform
    tf2::Transform old_trans, new_trans;
    tf2::convert(last_transform_, old_trans);
    tf2::convert(output_transform_->transform, new_trans);
    tf2::Transform diff = old_trans.inverseTimes(new_trans);
    
    geometry_msgs::TransformStamped gm_diff;
    tf2::convert(diff, gm_diff.transform);
    gm_diff.header = pcl_conversions::fromPCL(cloud1->header);
    gm_diff.header.frame_id = child_frame;
    gm_diff.child_frame_id = child_frame;

    tf2::Vector3 diff_vector(diff.getOrigin());
    tf2::Matrix3x3 diff_matrix(diff.getRotation());
    double diff_x, diff_y, diff_z, diff_roll, diff_pitch, diff_yaw;
    diff_x = diff_vector.getX();
    diff_y = diff_vector.getY();
    diff_z = diff_vector.getZ();
    diff_matrix.getRPY(diff_roll, diff_pitch, diff_yaw);

    //writeout values
    if(freeze0_) 
    {
      ROS_INFO_STREAM_NAMED(node_name, "input0 frozen");
    }
    if(freeze1_) 
    {
      ROS_INFO_STREAM_NAMED(node_name, "input1 frozen");
    }

    double roll,pitch,yaw;

    if (!lidar_to_robot_) {
      tf2::Quaternion q(
            output_transform_->transform.rotation.x,
            output_transform_->transform.rotation.y,
            output_transform_->transform.rotation.z,
            output_transform_->transform.rotation.w);
      tf2::Matrix3x3 m(q);
      m.getRPY(roll,pitch,yaw);
    }
    else 
    {
      calculateYaw();
      calculateRollPitchCorrection();

      ROS_INFO_STREAM_NAMED(node_name, "TF Translation  X: " << xTF_ << " m");
      ROS_INFO_STREAM_NAMED(node_name, "TF Translation  Y: " << yTF_ << " m");
      ROS_INFO_STREAM_NAMED(node_name, "TF Translation  Z: " << zTF_ << " m");
    
      roll  = current_roll_;
      pitch = current_pitch_;
      yaw   = current_yaw_;

      diff_roll  = alignPubConfig_.roll - roll;
      diff_pitch = alignPubConfig_.pitch - pitch;
      diff_yaw   = alignPubConfig_.yaw - yaw;
    }

    ROS_INFO_STREAM_NAMED(node_name, "ICP Translation X: " << output_transform_->transform.translation.x << " m" << ", diff: " << diff_x << " m");
    ROS_INFO_STREAM_NAMED(node_name, "ICP Translation Y: " << output_transform_->transform.translation.y << " m" << ", diff: " << diff_y << " m");
    ROS_INFO_STREAM_NAMED(node_name, "ICP Translation Z: " << output_transform_->transform.translation.z << " m" << ", diff: " << diff_z << " m");

    ROS_INFO_STREAM_NAMED(node_name, "Roll:  " << roll <<  " rad, " << (roll/PI*180)  << " deg" << ", diff: " << diff_roll << " rad");
    ROS_INFO_STREAM_NAMED(node_name, "pitch: " << pitch << " rad, " << (pitch/PI*180) << " deg" << ", diff: " << diff_pitch << " rad");
    ROS_INFO_STREAM_NAMED(node_name, "Yaw:   " << yaw <<   " rad, " << (yaw/PI*180)   << " deg" << ", diff: " << diff_yaw << " rad");

    // Create output msgs
    pcl::PointCloud<PointT>::Ptr output_cloud0(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr output_cloud1(new pcl::PointCloud<PointT>);

    if(is_output_filtered_)
    {
      pcl::copyPointCloud (*filtered_cloud0, *output_cloud0);
      pcl::copyPointCloud (*filtered_cloud1, *output_cloud1);
    }
    else
    {
      pcl::copyPointCloud (*cloud0, *output_cloud0);
      pcl::copyPointCloud (*cloud1, *output_cloud1);
    }

    geometry_msgs::TransformStamped::Ptr output(output_transform_);
    sensor_msgs::PointCloud2::Ptr output_msg0(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2::Ptr output_msg1(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2 p_msg1;
    pcl::toROSMsg(*output_cloud0, *output_msg0);
    pcl::toROSMsg(*output_cloud1, *output_msg1);

    //first convert output_msg1 to parent_frame then apply output transform
    // geometry_msgs::TransformStamped pTransform = tfBuffer_.lookupTransform(parent_frame, output_msg1->header.frame_id, output_msg1->header.stamp);
    // tf2::doTransform(*output_msg1, *output_msg1, pTransform);
    // tf2::doTransform(*output_msg1, *output_msg1, *output_transform_);

    // correct error in cloud1's output msg 
    tf2::doTransform(*output_msg1, *output_msg1, gm_diff);

    output_trans_pub_.publish(output);
    output_cloud0_pub_.publish(output_msg0);
    output_cloud1_pub_.publish(output_msg1);
  }

  void Cloud_Alignment::input0_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    
    if(freeze0_) 
    {
      return;
    }
    
    sensor_msgs::PointCloud2 cloud_in(*msg), cloud_out;
    geometry_msgs::TransformStamped transform;

    if(parent_frame_id_ != "")
    {
      try{
        transform = tfBuffer_.lookupTransform(parent_frame_id_, msg->header.frame_id, msg->header.stamp);
        
        tf2::doTransform(*msg, cloud_out, transform);

        cloud0_ = cloud_out;
        
        if(freeze1_) 
        {
          cloud0_.header = cloud1_.header;
        }
      }
      catch (tf2::TransformException ex){
        ROS_WARN("%s",ex.what());
        return;
      }
    }

  }

  void Cloud_Alignment::input1_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    stamp_ = msg->header.stamp;

    if(freeze1_) 
    {
      return;
    }

    sensor_msgs::PointCloud2 cloud_in(*msg), cloud_out;
    geometry_msgs::TransformStamped transform;

     if(child_frame_id_ != "")
    {
      try{
        transform = tfBuffer_.lookupTransform(child_frame_id_, msg->header.frame_id, msg->header.stamp);
        
        tf2::doTransform(*msg, cloud_out, transform);

        cloud1_ = cloud_out;

        if(freeze0_) 
        {
          cloud1_.header = cloud0_.header;
        }
      }
      catch (tf2::TransformException ex){
        ROS_WARN("%s",ex.what());
        return;
      }
    }

  }

  bool Cloud_Alignment::freeze0_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    freeze0_ = true;

    return true;	  
  }

  bool Cloud_Alignment::freeze1_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    freeze1_ = true;	  
    
    return true;
  }

  bool Cloud_Alignment::unfreeze0_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    ROS_INFO_STREAM_NAMED(node_name, "input0 unfrozen");
    freeze0_ = false;	  

    return true;
  }

  bool Cloud_Alignment::unfreeze1_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    ROS_INFO_STREAM_NAMED(node_name, "input1 unfrozen");
    freeze1_ = false;	  
    
    return true;
  }

  bool Cloud_Alignment::reset()
  {
    alignPubConfig_ = initialAlignPubConfig_;
    return alignClient_->setConfiguration(alignPubConfig_);
  }
  
  bool Cloud_Alignment::revert()
  {
    //revert Guess
    last_transform_ = tfBuffer_.lookupTransform(parent_frame_id_, child_frame_id_, ros::Time::now()).transform;
    
    Eigen::Affine3d eigenTransform = tf2::transformToEigen(last_transform_);
    current_guess_ = eigenTransform.matrix().cast<float>();
    
    //revert rolling window accumulators
    x_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    y_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    z_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qx_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qy_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qz_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qw_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    current_qx_ = 0; current_qy_ = 0; current_qz_ = 0; current_qw_ = 0; 
    
    ROS_INFO_STREAM_NAMED(node_name, "Guess transform reverted");

    return true;
  }

  bool Cloud_Alignment::revert_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Cloud_Alignment::revert();
  }

  bool Cloud_Alignment::reset_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Cloud_Alignment::reset();
  }

  bool Cloud_Alignment::pushtransform_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Cloud_Alignment::pushTransform();
  }
  
  bool Cloud_Alignment::pushYaw_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Cloud_Alignment::pushYaw();
  }
  
  bool Cloud_Alignment::pushRollPitchCorrection_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Cloud_Alignment::pushRollPitchCorrection();
  }
  
    
}  // namespace Multi_Sensor_Alignment





