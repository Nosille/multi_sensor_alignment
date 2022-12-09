/** ROS node 

/* 

Copyright (c) 2017

*/

#ifndef CB_ALIGN_TOOL_H
#define CB_ALIGN_TOOL_H

#include "sensor_alignment.h"

#include <mutex>

#include <ros/ros.h>
#include <ros/console.h>

#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <multi_sensor_alignment/cb_align_toolConfig.h>
#include <dynamic_reconfigure/client.h>
#include <multi_sensor_alignment/alignment_publisherConfig.h>
#include <visualization_msgs/Marker.h>

#include "icp_common_tools.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

using namespace boost::accumulators;
/**
 * \brief Cross-registers camera and pointcloud2 topics for use in alignment
 */

namespace Multi_Sensor_Alignment
{
  class Chessboard_Alignment
  {
  typedef accumulator_set<double, stats<tag::rolling_mean > > window_acc;
  
  public:
    Chessboard_Alignment(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle, int buffer_size);

    /** Destructor */
    ~Chessboard_Alignment();

    /** 
     * Main run loop
     */
    void onInit();

    //! Callback
    void reconfigure_server_callback(multi_sensor_alignment::cb_align_toolConfig &config, uint32_t level);
    void align_pubconfig_callback(const multi_sensor_alignment::alignment_publisherConfig& config);
    void align_pubdesc_callback(const dynamic_reconfigure::ConfigDescription& description);
    
    void input_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void input_camera_callback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& info_msg);
    
    //publisher
    void publish_callback(const ros::TimerEvent& event);

    std::string node_name{"chessboard_alignment"};

  private:
    double PI = atan(1)*4;
    
    //Methods
    bool reset();
    bool pushTransform();
    void ImageProcessing(cv::Mat &grey, std::vector<cv::Point2f> &chessCorners, pcl::PointCloud<PointT>::Ptr &cloud);
    void CloudProcessing(pcl::PointCloud<PointT>::Ptr &in_cloud);
    double * convert_to_imgpts(double x, double y, double z);
    // static bool compareLineCoeff(const pcl::ModelCoefficients::Ptr &c1, const pcl::ModelCoefficients::Ptr &c2);
    // static bool comparePoints(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);

    //service callbacks
    bool reset_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    bool pushtransform_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    // ROS 
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string parent_frame_id_, child_frame_id_;
    std::string output_trans_topic_;
    std::string output_cloud_topic_, output_camera_topic_, output_marker_topic_;
    std::string image_cloud_topic_;
    std::string filter_cloud_topic_;
    std::string align_server_name_;
    ros::Publisher output_trans_pub_, output_cloud_pub_, output_camera_pub_, output_marker_pub_;
    ros::Publisher image_cloud_pub_, filter_cloud_pub_;

    tf2_ros::TransformListener tfListener_;
    tf2_ros::Buffer tfBuffer_;
    float wait_for_tf_delay_;
    bool is_rectified_;

    boost::shared_ptr<dynamic_reconfigure::Server<multi_sensor_alignment::cb_align_toolConfig> > drServer_;
    boost::shared_ptr<dynamic_reconfigure::Client<multi_sensor_alignment::alignment_publisherConfig> > alignClient_;

    multi_sensor_alignment::alignment_publisherConfig alignPubConfig_;
    multi_sensor_alignment::cb_align_toolConfig alignToolConfig_;
    dynamic_reconfigure::ConfigDescription alignPubDesc_;

    bool received_alignToolConfig_;
    bool received_alignPubConfig_;
    bool received_alignPubDesc_;

    std::string input_cloud_topic_, input_image_topic_, input_info_topic_;
    Multi_Sensor_Alignment::SourceCamera* sourceImage_;
    Multi_Sensor_Alignment::SourceCloud* sourceCloud_;

    int grid_rows_, grid_cols_;
    double square_size_, board_height_, board_width_, height_offset_, width_offset_;

    ros::ServiceServer service0_, service1_;

    int buffer_size_;

    double x_, y_, z_, roll_, pitch_, yaw_;

    Eigen::Matrix4f current_guess_;
    geometry_msgs::TransformStamped::Ptr output_transform_;
    geometry_msgs::TransformStamped::Ptr image_transform_;
    geometry_msgs::TransformStamped::Ptr cloud_transform_;
    window_acc x_array_;
    window_acc y_array_;
    window_acc z_array_;
    window_acc qw_array_;
    window_acc qx_array_;
    window_acc qy_array_;
    window_acc qz_array_;
    double current_qw_, current_qx_, current_qy_, current_qz_;

    std::mutex cloud_mutex_;
    std::mutex camera_mutex_;
    boost::recursive_mutex drServer_mutex_;
    ros::Timer pub_timer_;
    double output_frequency_;
  
  }; // class Chessboard_Alignment
} // namespace Multi_Sensor_Alignment

#endif  // CB_ALIGN_TOOL_H

