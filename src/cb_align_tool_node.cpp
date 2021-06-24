#include "cb_align_tool/cb_align_tool.h"
#include <ros/callback_queue.h>

int main(int argc, char **argv)
{

  std::string node_name = "cb_align_tool_node";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  
  ROS_INFO_STREAM_NAMED(node_name, "cb_align_tool node starting.");

  double control_frequency;
  nh_private.param<double>("output_frequency", control_frequency, 10.0);
      ROS_INFO_STREAM_NAMED(node_name, "output frequency set to " << control_frequency);      

  int buffer_size;
  nh_private.param("buffer_size", buffer_size, 1);
      ROS_INFO_STREAM_NAMED(node_name, "buffer_size set to " << buffer_size);

  Multi_Sensor_Alignment::Chessboard_Alignment node(nh, nh_private, buffer_size);

  //Setup spinner for subscribers
  ros::MultiThreadedSpinner sub_spinner(5); // Use 5 threads
  sub_spinner.spin(); // spin() will not return until the node has been shutdown

  return 0;
}

