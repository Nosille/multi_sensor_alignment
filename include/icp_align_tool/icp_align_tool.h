/** ROS node 

/* 

Copyright (c) 2017

*/

#ifndef ICP_ALIGN_TOOL_H
#define ICP_ALIGN_TOOL_H

#include <iostream>
#include <vector> 
#include <sstream>

#include <mutex>

#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
// #include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <multi_sensor_alignment/icp_align_toolConfig.h>
#include <dynamic_reconfigure/client.h>
#include <multi_sensor_alignment/alignment_publisherConfig.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include "icp_common_tools.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "std_msgs/String.h"

/**
 * \brief Cross-registers pointcloud2 topics for use in alignment
 */

namespace Multi_Sensor_Alignment
{
  class Cloud_Alignment
  {
    
  public:
    Cloud_Alignment(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle, int buffer_size);

    /** Destructor */
    ~Cloud_Alignment();

    /** 
     * Main run loop
     */
    void onInit();

    //! Callback
    void reconfigure_server_callback(multi_sensor_alignment::icp_align_toolConfig &config, uint32_t level);
    void align_pubconfig_callback(const multi_sensor_alignment::alignment_publisherConfig& config);
    void align_pubdesc_callback(const dynamic_reconfigure::ConfigDescription& description);
    
    void input0_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void input1_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    void initSensorFixedFrame();

    //publisher
    void publish_callback(const ros::TimerEvent& event);
    
    ros::Timer pub_sensor_fixed_frame_timer_;

    std::string node_name{"cloud_alignment"};

  private:
    double PI = atan(1)*4;
    
    //Methods
    bool revert();
    bool reset();
    bool pushTransform();
    bool calculateYaw();
    bool pushYaw();
    bool calculateRollPitchCorrection();
    bool pushRollPitchCorrection();

    //service callbacks
    bool freeze0_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool freeze1_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool unfreeze0_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool unfreeze1_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool revert_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool reset_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool pushtransform_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool pushYaw_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool pushRollPitchCorrection_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    
    // ROS 
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    bool lidar_to_robot_;

    std::string parent_frame_id_, child_frame_id_, fixed_frame_id_, base_link_frame_id_, fixed_sensor_frame_id_, original_parent_frame_id_;
    std::string output_trans_topic_;
    std::string output_cloud0_topic_, output_cloud1_topic_;
    std::string align_server_name_;
    ros::Publisher output_trans_pub_,  output_cloud0_pub_, output_cloud1_pub_;

    geometry_msgs::TransformStamped sensorFixedFrameTransform_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::Buffer tfBuffer_;
    float wait_for_tf_delay_;

    boost::shared_ptr<dynamic_reconfigure::Server<multi_sensor_alignment::icp_align_toolConfig> > drServer_;
    boost::shared_ptr<dynamic_reconfigure::Client<multi_sensor_alignment::alignment_publisherConfig> > alignClient_;

    multi_sensor_alignment::alignment_publisherConfig initialAlignPubConfig_;
    multi_sensor_alignment::alignment_publisherConfig alignPubConfig_;
    multi_sensor_alignment::icp_align_toolConfig alignToolConfig_;
    dynamic_reconfigure::ConfigDescription alignPubDesc_;

    bool received_alignToolConfig_;
    bool received_alignPubConfig_;
    bool received_alignPubDesc_;

    std::string input0_topic_, input1_topic_;
    sensor_msgs::PointCloud2 cloud0_, cloud1_;
    ros::Subscriber input_sub0_, input_sub1_;
    ros::ServiceServer service0_, service1_, service2_, service3_, service4_, service5_, service6_, service7_, service8_;
    bool freeze0_, freeze1_, is_output_filtered_;

    ros::Time stamp_;

    int buffer_size_;

//     int method_;
//     int    norm_kSearch_;
//     double norm_RadiusSearch_;

//     double epsilon_;
//     int    maxIterations_;
//     double maxCorrespondenceDistance_;
    
//     double ndt_StepSize_;
//     double ndt_Resolution_;

//     double voxelSize_;
//     double filter_i_min_;
//     double filter_i_max_;
//     double filter_x_min_;
//     double filter_x_max_;
//     double filter_y_min_;
//     double filter_y_max_;
//     double filter_z_min_;
//     double filter_z_max_;

    Eigen::Matrix4f current_guess_;
    geometry_msgs::TransformStamped::Ptr output_transform_;
    geometry_msgs::Transform last_transform_;
    window_acc x_array_;
    window_acc y_array_;
    window_acc z_array_;
    window_acc qw_array_;
    window_acc qx_array_;
    window_acc qy_array_;
    window_acc qz_array_;
    double current_qw_, current_qx_, current_qy_, current_qz_;

    double current_yaw_, current_pitch_, current_roll_;
    double xTF_, yTF_, zTF_;

    std::recursive_mutex cloud0_mutex_;
    std::recursive_mutex cloud1_mutex_;
    boost::recursive_mutex drServer_mutex_;
    ros::Timer pub_timer_;
    double output_frequency_;
   
  }; // class Cloud_Alignment
} // namespace Multi_Sensor_Alignment

#endif  // ICP_ALIGN_TOOL_H

