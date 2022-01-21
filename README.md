# Multi Sensor Alignment

`multi_sensor_alignment` is a collection of ROS tools to aid with sensor alignment for multi-sensor setups.  A list of the various exectubles follows:

	alignment_publisher     #Maintains a static transform between two tf2 frames while providing realtime adjustment capability through a dynamic parameter server.
	icp_align_tool          #Subscribes to two pointcloud2 messages and provides the best fit transform for aligning them.  Can connect to a running alignment publisher for automatic realtime adjustments. 


## Overview

By combining the data from multiple sensors, we can theoretically improve many aspects of our maps; redundancy, resolution, accuracy, range, and provide improved perception and object identification.  However, the benefits are contingent on limiting the additional errors introduced by the process of fusing the data sources. One big source of error is differences in the assumed sensor perspective used to fuse the data and the actual relative location and orientation of each sensor.  In ROS, the [tf2 package](http://wiki.ros.org/tf2) is widely used to handle transforming data from one sensor frame to another, but the results are only as good the information contained in the underlying model.  The multi_sensor_alignment package provides tools for aligning those sensors frames in real time.

## alignment_publisher

A major source of error in the fusion process is potential misalignments between the physical sensors and our computational models.  Traditional methods of modifying the models can be tedious and time-consuming.  The alignment_publisher node is designed to address this issue with a way to quickly realign sensors in real-time.

This is accomplished using a ROS node that combines two widely used ROS features, a static transform publisher based on the [tf2 package](http://wiki.ros.org/tf2) and a dynamic parameter server based on the [dynamic reconfigure package](http://wiki.ros.org/dynamic_reconfigure). The tf2 package provides relative location information for all points in the robot model and services for defining, obtaining, and changing the transforms used to move between points on the robot.  A static transform publisher was chosen because of it's latching mechanism limits message traffic (messages are only published when a change is made, with clients continuing to use the last transform until an a new update is published in contrast with traditional dynamic transforms that are continuously published at a set interval).  In the alignment publisher, a frame between the sensor and the robot can be initialized from a yaml file on startup of the alignment publisher node and then later overwritten (or not) as changes are required.  The alignment_publisher provides a dynamic parameter server for these quick, on the fly, configuration changes which are published as a new static transform when they occur.  Changes to the dynamic parameter server can be made by other ROS nodes setup as clients of the dynamic parameter server or manually using existing GUI interfaces such as the rqt package http://wiki.ros.org/rqt).

On startup the alignment_publisher, will take parameters defining a static transform (can be read from a yaml file), publish those values to the dynamic parameter and static transform servers, then monitor the dynamic parameter servers for changes, republishing the transform whenever a change occurs.  A service is also provided that will write the current parameters back out to the yaml file achieving persistence between runs.  The dynamic parameter server provides realtime control of all six degrees of freedom in the form of x, y, z, roll, pitch, and yaw values, while the launch parameters include these six degrees of freedom plus the parent and child reference frames required to completely define the transform.  

A list of all input parameters follows:

	alignment_file			File to save to when save_joint_state service is called
	parent_frame			Transform parent
	child_frame				Transform child
	x						initial x displacement of transform
	y						initial y displacement of transform
	z						initial z displacement of transform
	roll					initial roll of transform
	pitch					initial pitch of transform
	yaw					initial yaw of transform

The alignment_file parameter defines where the save_joint state service, "{namespace}/{node_name}/save_joint_state", will save the file when called.  Ideally, the user will load an alignment_file on node startup and save to that same file on the service call, achieving persistance between runs. See the example launch file "launch/alignment_publisher.launch" for this approach.  An example yaml file is also provided "config/sensor_alignment/sensor_joint_state".

### Discussion and Examples

It is possible to use rqt and a visualization tool such as rviz to manually tweek the alignment of any sensor configured with the Alignment Publisher.  For some simple alignments this can be the easiest solution.  We typically check the alignment on our imu's by starting "rosrun tf tf_echo base_link odom" to gain realtime status of the gravity vector, record the roll/ pitch/yaw orientation values produced, perform a 180 deg spin on a hard flat surface, and compare the final roll/pitch/yaw values to the initial ones.  Yaw values should confirm the 180 deg spin, and the roll and pitch values should show a sign reversal between intial and final values if the imu is properly aligned.  Any deviation is a misalignment.  If significant, the alignment error can be quickly corrected using the dynamic parameter server.  For a simple ground truth, the average between the initial and final poses for both roll and pitch should be zero, so the averages can be added to the existing transform as a correction.  Don't forget to perform a "rosservice call {namespace}/{alignment_publisher_name}/save_joint_state" when satisfied with the alignment to make it persistent.

## icp_align_tool

This node calculates a real time transform between two pointclouds using one of 3 registration method:

   1) [pcl::IterativeClosestPointNonLinear](https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point_non_linear.html)
   2) [pcl::NormalDistributionsTransform](https://pointclouds.org/documentation/classpcl_1_1_normal_distributions_transform.html)
   3) [pcl::IterativeClosestPointWithNormals](https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point_with_normals.html)

Static input parameters include:
	
	output_frequency		frequency target for output transform
	buffer_size				smoothing window for averaging the transform
	alignment_server        namespaced name for aligment_server, will provide direct dynamic reconfiguration of an alignment_server node

	input_cloud0			Firstpointcloud2 topic for registration, target
	input_cloud1			Second pointcloud2 topic for registration, source
	
	parent_frame			Working frame for cloud0, "" will use frame in message header
	child_frame				Working frame for cloud1, "" will use frame in message header
	x						x displacement guess between parent and child frames 
	y						y displacement guess between parent and child frames
	z						z displacement guess between parent and child frames
	roll					roll guess between parent and child frames
	pitch					ptich guess between parent and child frames
	yaw						yaw guess between parent and child frames

	output					topic name for publishing the transform
	output_cloud0			topic name for republishing cloud0
	output_cloud1			topic name for republishing cloud1, after transformation
	is_output_filtered		if false full input clouds will be republished, if true output will reflect all filters applied
    

The node will transform cloud0 into the parent frame and cloud1 into the child frame and calculate all transforms between those two frames.  The user will need to provide initial x, y, z, roll, pitch, and yaw guess values for the transform between these two frames.  Ideally, this is accomplished by loading the same startup yaml file used by a companion alignment_publisher, see example launch files "launch/alignment_publisher.launch" and "launch/icp_align_tool.launch".  This approach will allow this node to work in the same frames as the alignment_publisher making all transforms a one to one comparison.  The alignment_server parameter takes this approach ones step further with two-way interactions between this node and the alignment_server.  Alternatively, setting parent_frame=child_frame and all 6 degrees of freedom to zero will work in the absense of an aligment_publisher.  In this alternative approach, the transform simply represents the needed tf correction (error) implied by the icp registration instead of a full parent-child transform.

Further capabilities provided through the alignment_server parameter, include automatic reset of the icp_align_tool node after an outside change to the alignment publishers's dynamic parmeter server values, and a service that will push the current icp registration to the alignment_publisher as the new transform (i.e., immediate use on the robot by all tf subscribers).

After startup, each transform solution uses the previous solution as its initial guess.  Results are written to the screen in the form of ROS_INFO_STREAM messages and published within ROS as [geometry_msgs::TransformStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TransformStamped.html).  Output also includes an echo of the target pointcloud (cloud0) in the parent_frame and the source pointcloud (cloud1) in the same frame with the proposed transformation applied. The node includes filters to remove unwanted points from the pointcloud based on intensity or point coordinate (x/y/z in the parent/child frames) with control over the filters and most of the settings related to cloud registration available through a dynamic parameter server: 

	filter/i_min			pointcloud filter on minimum intensity
	filter/i_max			pointcloud filter on maximum intensity
	filter/x_min			pointcloud filter on minimum x coordinate in parent/child frame
	filter/x_max			pointcloud filter on maximum x coordinate in parent/child frame
	filter/y_min			pointcloud filter on minimum y coordinate in parent/child frame
	filter/y_max			pointcloud filter on maximum y coordinate in parent/child frame
	filter/z_min			pointcloud filter on minimum z coordinate in parent/child frame
	filter/z_max			pointcloud filter on maximum z coordinate in parent/child frame
	voxelSize			pointcloud voxel size used to filter pointclouds
	method				pointcloud registration method
	epsilon				epsilon, used by all registration methods
	maxIterations			max iterations, used by all registration methods
	maxCorrespondenceDistance	max correspondence distance, used by all icp registration methods
	norm/KSearch			number of points used to calculate normals, used by all icp registration methods
	norm/RadiusSearch		radius used to calculate normals, used by all icp registration methods, only used when KSearch is 0
	ndt/StepSize			step size, used by ndt registration method
	ndt/Resolution			resolution, used by ndt registration method

The node also provides a few additional services as shown below:

	freeze_cloud0		Blocks update of cloud0.  The last cloud received before calling this service will continue to be used in the transformation, indefinitely.
	freeze_cloud1		Blocks update of cloud1.  The last cloud received before calling this service will continue to be used in the transformation, indefinitely.
	unfreeze_cloud0		Reenables cloud updates on cloud0.
	unfreeze_cloud1		Reenables cloud updates on cloud1.
	reset_guess		Resets buffers and reverts the initial guess used for the next registration to the initial value provided at launch (or current value in a companion alignment_publisher).
	push_transform		Push current transform to the alignment_publisher for immediate use on the robot (requires node name for an alignment_publisher to be provided at launch).

Do not forget to use the save_joint_state service in the alignment publisher, "{namespace}/{node_name}/save_joint_state", to achieve transform persistence through a restart of the alignment publisher.

### Discussion and examples

The authors have been using the multi_sensor_alignment package to maintain a robot using three Velodyne VLP-16 lidar.  We treat one lidar as a master lidar and align the other two with it.  All three lidar have an alignment_joint, each maintained by a separate alignment publisher, running at all times.

The icp_align_tool node has proven very useful for aligning the slave lidar with the master.  This process can occur very quickly in the field with the robot at rest.  The author has had the best luck with registration method 3  (IterativeClosesPointWithNormals) using the settings in the example launch file, "launch/icp_align_tool.launch", but your mileage may vary.  Method 3 is a good choice when multiple large flat surfaces are located in the area where the sensors overlap (e.g., the corner of a building with two walls and a ground plane avaliable for the plane-to-plane matching provided by IteractiveClosesPointWithNormals).  The process consists of launching an icp_align_tool node that is subscribing to the master lidar on input_cloud0 and the slave lidar on input_cloud1, monitor the correction in rviz by displaying the output_cloud0 and output_cloud1 output messages, possibly adjust filters and registration parameters on the dynamic parameter server to obtain a better transform, then once the users is satisfied with the transform provided "rosservice call {namespace}/{icp_align_tool_name}/push_transform" will cause the robot to begin using the revised transform.  Shutdown the icp_align_tool node, quickly confirm that the revised transforms are producing good results, and "rosservice call {namespace}/{alignment_publisher_name}/save_joint_state" to make the new transforms persistent.

This node can also be used effectively to align a single lidar with the robot, but with additional effort on the operators part.  The process consists of subscribing both input clouds to the same pointcloud2 message, parent_frame = child_frame = "odom", and capturing two poses using the freeze_cloud service calls.  Use "rosservice call {namespace}/{icp_align_tool_name}/freeze_cloud0" to capture the first pose, drive the robot, use "rosservice call {namespace}/{icp_align_tool_name}/freeze_cloud1" to capture a second pose, then compare the resulting transform to a known ground truth, and manually update the sensor alignment accordingly.  This approach can easily be used to level a lidar when pose 2 (cloud1) is the result of a 180 deg spin on a hard flat surface relative to pose 1 (cloud0).  In this scenario, the ground truth is the average of the two poses, so 1/2 the pitch difference and 1/2 the roll difference can be manually applied using rqt to access the Alignment Publisher's dynamic parameter server.  None of the other tranform parameters mean much in this scenario beyond confirming the accuracy of the in place spin (i.e., yaw = 180 deg).  A separate yaw correction can be similarly obtained by driving the robot a short distance in a straight line along odom x-axis and comparing a pose at the beginning of the drive to a pose at the end.  Any y drift in the resulting transform should be corrected (i.e., yaw correction=atan(y/x)).  

## Changelog

- 


