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

#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <multi_sensor_alignment/icp_align_toolConfig.h>
#include <dynamic_reconfigure/client.h>
#include <multi_sensor_alignment/alignment_publisherConfig.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/ndt.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#include "std_msgs/String.h"

using namespace boost::accumulators;

//convenient typedefs
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

/**
 * \brief Cross-registers pointcloud2 topics for use in alignment
 */

namespace Multi_Sensor_Alignment
{
  class Cloud_Alignment
  {
  typedef accumulator_set<double, stats<tag::rolling_mean > > window_acc;
  
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
    
    //publisher
    void publish_callback(const ros::TimerEvent& event);

    std::string node_name{"cloud_alignment"};

  private:
    double PI = atan(1)*4;
    
    //Methods
    bool reset();
    bool pushTransform();

    static bool AreQuaternionsClose(tf2::Quaternion q1, tf2::Quaternion q2);
    geometry_msgs::Quaternion AverageQuaternion(const geometry_msgs::Quaternion& newRotation);
    void DownsampleCloud(const pcl::PointCloud<PointT>::Ptr in_cloud, pcl::PointCloud<PointT> &out_cloud, double in_leaf_size);

    //service callbacks
    bool freeze0_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool freeze1_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool unfreeze0_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool unfreeze1_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool reset_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    bool pushtransform_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp);
    
    // ROS 
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string parent_frame_id_, child_frame_id_;
    std::string output_trans_topic_;
    std::string output_cloud0_topic_, output_cloud1_topic_;
    std::string align_server_name_;
    ros::Publisher output_trans_pub_,  output_cloud0_pub_, output_cloud1_pub_;

    tf2_ros::TransformListener tfListener_;
    tf2_ros::Buffer tfBuffer_;
    float wait_for_tf_delay_;

    boost::shared_ptr<dynamic_reconfigure::Server<multi_sensor_alignment::icp_align_toolConfig> > drServer_;
    boost::shared_ptr<dynamic_reconfigure::Client<multi_sensor_alignment::alignment_publisherConfig> > alignClient_;

    multi_sensor_alignment::alignment_publisherConfig alignPubConfig_;
    multi_sensor_alignment::icp_align_toolConfig alignToolConfig_;
    dynamic_reconfigure::ConfigDescription alignPubDesc_;

    bool received_alignToolConfig_;
    bool received_alignPubConfig_;
    bool received_alignPubDesc_;

    std::string input0_topic_, input1_topic_;
    sensor_msgs::PointCloud2 cloud0_, cloud1_;
    ros::Subscriber input_sub0_, input_sub1_;
    ros::ServiceServer service0_, service1_, service2_, service3_, service4_, service5_;
    bool freeze0_, freeze1_, is_output_filtered_;

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
    geometry_msgs::TransformStamped::Ptr output_;
    geometry_msgs::Transform last_transform_;
    window_acc x_array_;
    window_acc y_array_;
    window_acc z_array_;
    window_acc qw_array_;
    window_acc qx_array_;
    window_acc qy_array_;
    window_acc qz_array_;
    double current_qw_, current_qx_, current_qy_, current_qz_;

    std::recursive_mutex cloud0_mutex_;
    std::recursive_mutex cloud1_mutex_;
    boost::recursive_mutex drServer_mutex_;
    ros::Timer pub_timer_;
    double output_frequency_;
   
  }; // class Cloud_Alignment
} // namespace Multi_Sensor_Alignment

#endif  // ICP_ALIGN_TOOL_H

