/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <multi_sensor_alignment/alignment_publisherConfig.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <std_srvs/Empty.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

static std::string lidar_id_str_;
static std::string child_frame_;
static std::string parent_frame_;
static std::string alignment_file_;


boost::recursive_mutex server_mutex_;
boost::shared_ptr<dynamic_reconfigure::Server<multi_sensor_alignment::alignment_publisherConfig>> server_;
multi_sensor_alignment::alignment_publisherConfig pose_, initialPose_;

namespace Multi_Sensor_Alignment
{
  bool save_params_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
  {

    // build YAML document
    YAML::Emitter yaml;
      yaml << YAML::BeginMap;
      yaml << YAML::Key << "parent_frame" << YAML::Value << parent_frame_;
      yaml << YAML::Key << "child_frame" << YAML::Value << child_frame_;
      yaml << YAML::Key << "x" << YAML::Value << pose_.x;
      yaml << YAML::Key << "y" << YAML::Value << pose_.y;
      yaml << YAML::Key << "z" << YAML::Value << pose_.z;
      yaml << YAML::Key << "roll" << YAML::Value << pose_.roll;
      yaml << YAML::Key << "pitch" << YAML::Value << pose_.pitch;
      yaml << YAML::Key << "yaw" << YAML::Value << pose_.yaw;
      yaml << YAML::EndMap;

    // write to file
    try
    {
      std::ofstream fout;
      fout.open((alignment_file_).c_str());
      fout << yaml.c_str();
      fout.close();
    }
    catch(...)
    {
      ROS_ERROR("Cannot save lidar alignment file.");
      return false;
    }
    
    return true;
  }

  void tfRegistration(const ros::Time &timeStamp)
  {
    static tf2_ros::StaticTransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = timeStamp;
    transformStamped.header.frame_id = parent_frame_;
    transformStamped.child_frame_id = child_frame_;
    transformStamped.transform.translation.x = pose_.x;
    transformStamped.transform.translation.y = pose_.y;
    transformStamped.transform.translation.z = pose_.z;
    tf2::Quaternion q;
    q.setRPY(pose_.roll, pose_.pitch, pose_.yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    
    ROS_DEBUG("x=%f y=%f z=%f",  transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    broadcaster.sendTransform(transformStamped);

  }

  void reconfigure_callback(multi_sensor_alignment::alignment_publisherConfig &config, uint32_t level) 
  {
    
    ROS_INFO("Reconfigure Request: %f %f %f %f %f %f", 
            config.x, config.y, config.z, config.roll, config.pitch, config.yaw);

    pose_.x     = config.x;
    pose_.y     = config.y;
    pose_.z     = config.z;
    pose_.roll  = config.roll;
    pose_.pitch = config.pitch;
    pose_.yaw   = config.yaw;

    if(parent_frame_ != "" || child_frame_ != "")  tfRegistration(ros::Time(0));
  }

  bool revert_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) 
  {
    dynamic_reconfigure::Server<multi_sensor_alignment::alignment_publisherConfig> server;
    
    server_->updateConfig(initialPose_);
    reconfigure_callback(initialPose_, 0);

    return true;
  }
  
} // namespace Multi_Sensor_Alignment

int main(int argc, char *argv[])
  {
  ros::init(argc, argv, "simple_publisher_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");
  char __APP_NAME__[] = "sensor_alignment_simple_publisher_node";

  private_nh.param<std::string>("child_frame", child_frame_, "velodyne1");
  ROS_INFO("[%s] child_frame: '%s'", __APP_NAME__, child_frame_.c_str());

  private_nh.param<std::string>("parent_frame", parent_frame_, "velodyne0");
  ROS_INFO("[%s] parent_frame: '%s'", __APP_NAME__, parent_frame_.c_str());

  private_nh.param<std::string>("alignment_file", alignment_file_, "joint_state.yaml");
  ROS_INFO("[%s] alignment_file: '%s'", __APP_NAME__, alignment_file_.c_str());

  private_nh.param<double>("x", pose_.x, 0.0);
  ROS_INFO("[%s] x: %f", __APP_NAME__, pose_.x);

    private_nh.param<double>("y", pose_.y, 0.0);
  ROS_INFO("[%s] y: '%f'", __APP_NAME__, pose_.y);

    private_nh.param<double>("z", pose_.z, 0.0);
  ROS_INFO("[%s] z: '%f'", __APP_NAME__, pose_.z);

    private_nh.param<double>("roll", pose_.roll, 0.0);
  ROS_INFO("[%s] roll: '%f'", __APP_NAME__, pose_.roll);

    private_nh.param<double>("pitch", pose_.pitch, 0.0);
  ROS_INFO("[%s] pitch: '%f'", __APP_NAME__, pose_.pitch);

    private_nh.param<double>("yaw", pose_.yaw, 0.0);
  ROS_INFO("[%s] yaw: '%f'", __APP_NAME__, pose_.yaw);

  initialPose_ = pose_;

  //Setup Dynamic Reconfigure Server for alignCheckConfig
  dynamic_reconfigure::Server<multi_sensor_alignment::alignment_publisherConfig>::CallbackType
      serverCallback = boost::bind(&Multi_Sensor_Alignment::reconfigure_callback, _1, _2);
  server_.reset(new dynamic_reconfigure::Server<multi_sensor_alignment::alignment_publisherConfig>(server_mutex_, private_nh));
  server_->setCallback(serverCallback);

  std::string save_service_name = ros::this_node::getName() + "/save_joint_state";
  ros::ServiceServer save_service = node.advertiseService(save_service_name, Multi_Sensor_Alignment::save_params_callback);

  std::string revert_service_name = ros::this_node::getName() + "/revert_joint_state";
  ros::ServiceServer revert_service = node.advertiseService(revert_service_name, Multi_Sensor_Alignment::revert_callback);

  ros::Duration(0.1).sleep();
  Multi_Sensor_Alignment::tfRegistration(ros::Time(0));

  ROS_INFO("Spinning node");
  ros::spin();

  return 0;
}