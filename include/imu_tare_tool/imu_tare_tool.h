/** ROS node 

/* 

Copyright (c) 2017

*/

#ifndef IMU_TARE_TOOL_H
#define IMU_TARE_TOOL_H

#include <iostream>
#include <vector> 
#include <sstream>

#include <mutex>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <multi_sensor_alignment/imu_tare_toolConfig.h>
#include <dynamic_reconfigure/client.h>
#include <multi_sensor_alignment/alignment_publisherConfig.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#include "std_msgs/String.h"

using namespace boost::accumulators;

/**
 * \brief Tare an IMU by turning in a circle
 */

namespace Multi_Sensor_Alignment
{
  class IMU_Tare
  {
  typedef accumulator_set<double, stats<tag::rolling_mean > > window_acc;
  
  public:
    IMU_Tare(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle, int buffer_size);

    /** Destructor */
    ~IMU_Tare();

    /** 
     * Main run loop
     */
    void onInit();

    //! Callback
    void reconfigure_server_callback(multi_sensor_alignment::imu_tare_toolConfig &config, uint32_t level);
    void align_pubconfig_callback(const multi_sensor_alignment::alignment_publisherConfig& config);
    void align_pubdesc_callback(const dynamic_reconfigure::ConfigDescription& description);
    
    void input_callback(const sensor_msgs::Imu::ConstPtr& msg);

    //publisher
    void publish_callback(const ros::TimerEvent& event);
    
    ros::Timer pub_sensor_fixed_frame_timer_;

    std::string node_name{"IMU_Tare"};

  private:
    double PI = atan(1)*4;
    
    //Methods
    bool revert();
    bool reset();
    bool pushYawCorrection();
    bool pushRollPitchCorrection();

    static bool AreQuaternionsClose(tf2::Quaternion q1, tf2::Quaternion q2);
    geometry_msgs::Quaternion AverageQuaternion(const geometry_msgs::Quaternion& newRotation);

    //service callbacks
    bool capture_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool revert_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool reset_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool pushYawCorrection_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool pushRollPitchCorrection_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    
    // ROS 
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string parent_frame_id_, child_frame_id_, base_link_frame_id_;
//     std::string fixed_frame_id_;
    std::string output_trans_topic_;
    std::string align_server_name_;
    ros::Publisher output_trans_pub_;

    tf2_ros::TransformListener tfListener_;
    tf2_ros::Buffer tfBuffer_;
    float wait_for_tf_delay_;

    boost::shared_ptr<dynamic_reconfigure::Server<multi_sensor_alignment::imu_tare_toolConfig> > drServer_;
    boost::shared_ptr<dynamic_reconfigure::Client<multi_sensor_alignment::alignment_publisherConfig> > alignClient_;

    multi_sensor_alignment::alignment_publisherConfig initialAlignPubConfig_;
    multi_sensor_alignment::alignment_publisherConfig alignPubConfig_;
    multi_sensor_alignment::imu_tare_toolConfig tareToolConfig_;
    dynamic_reconfigure::ConfigDescription alignPubDesc_;

    bool received_tareToolConfig_;
    bool received_alignPubConfig_;
    bool received_alignPubDesc_;

    std::string input_topic_;
    ros::Subscriber input_sub_;
    ros::ServiceServer service0_, service1_, service2_, service3_, service4_;

    ros::Time stamp_;

    int buffer_size_;
    bool capture_;
    geometry_msgs::TransformStamped bs_to_world_transform_;

    Eigen::Matrix4f current_guess_;
    geometry_msgs::TransformStamped::Ptr output_;
    geometry_msgs::Transform last_transform_;
    window_acc qw_array_;
    window_acc qx_array_;
    window_acc qy_array_;
    window_acc qz_array_;
    double current_qw_, current_qx_, current_qy_, current_qz_;

    double current_yaw_, current_pitch_, current_roll_;
    double xTF_, yTF_, zTF_;

    boost::recursive_mutex drServer_mutex_;
    ros::Timer pub_timer_;
    double output_frequency_;
   
  }; // class IMU_Alignment
} // namespace Multi_Sensor_Alignment

#endif  // IMU_TARE_TOOL_H

