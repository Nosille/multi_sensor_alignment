
#include "cb_align_tool/cb_align_tool.h"

namespace Multi_Sensor_Alignment
{
  Chessboard_Alignment::Chessboard_Alignment(const ros::NodeHandle &node_handle, 
                        const ros::NodeHandle &private_node_handle, int buffer_size) 
  // Initialization list
  :nh_(node_handle),
  pnh_(private_node_handle),
  current_guess_(Eigen::Matrix4f::Identity()),
  output_transform_(new geometry_msgs::TransformStamped),
  image_transform_(new geometry_msgs::TransformStamped),
  cloud_transform_(new geometry_msgs::TransformStamped),
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
  Chessboard_Alignment::~Chessboard_Alignment()
  {
    // delete sourceCloud_;
    // delete sourceImage_;
  }

  void Chessboard_Alignment::onInit()
  {

    const std::string complete_ns = pnh_.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    node_name = complete_ns.substr(id + 1);

  //Setup Dynamic Reconfigure Server for this node
    dynamic_reconfigure::Server<multi_sensor_alignment::cb_align_toolConfig>::CallbackType
        drServerCallback_ = boost::bind(&Chessboard_Alignment::reconfigure_server_callback, this, _1, _2);
    drServer_.reset(new dynamic_reconfigure::Server<multi_sensor_alignment::cb_align_toolConfig>(drServer_mutex_, pnh_));
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
    alignClient_->setConfigurationCallback(boost::bind(&Chessboard_Alignment::align_pubconfig_callback, this, _1));
    alignClient_->setDescriptionCallback(boost::bind(&Chessboard_Alignment::align_pubdesc_callback, this, _1));
    
    //Wait up to 60 seconds for the alignment publisher nodes dynamic param server to respond
    int count = 0, maxcount = 60;
    while((count < maxcount && (!received_alignPubConfig_ || !received_alignPubDesc_)) && align_server_name_ != "")
    {
      ros::Duration(1.0).sleep();
      ROS_INFO_STREAM_NAMED(node_name, "Waiting on dynamic parameters from align_publisher. " << (maxcount-count) << " sec before giving up.");
      ros::spinOnce();
      count++;
    }

  // ROS Parameters
    pnh_.param<std::string>("parent_frame", parent_frame_id_, "");
      ROS_INFO_STREAM_NAMED(node_name, "parent_frame set to " << parent_frame_id_);
    pnh_.param<std::string>("child_frame", child_frame_id_, "");
      ROS_INFO_STREAM_NAMED(node_name, "child_frame set to " << child_frame_id_);
    pnh_.param("wait_for_tf_delay", wait_for_tf_delay_, wait_for_tf_delay_);
      ROS_INFO_STREAM_NAMED(node_name, "wait_for_tf_delay set to " << wait_for_tf_delay_);

    pnh_.param("grid_cols", grid_cols_, 9);
      ROS_INFO_STREAM_NAMED(node_name, "grid_cols set to " << grid_cols_);
    pnh_.param("grid_rows", grid_rows_, 6);
      ROS_INFO_STREAM_NAMED(node_name, "grid_rows set to " << grid_rows_);  
    pnh_.param("square_size_mm", square_size_, 60.0);
      ROS_INFO_STREAM_NAMED(node_name, "square_size set to " << square_size_);
    pnh_.param("board_height_mm", board_height_, 600.0);
      ROS_INFO_STREAM_NAMED(node_name, "board_height set to " << board_height_);
    pnh_.param("board_width_mm", board_width_, 420.0);
      ROS_INFO_STREAM_NAMED(node_name, "board_width set to " << board_width_);
    pnh_.param("height_offset_mm", height_offset_, 0.0);
      ROS_INFO_STREAM_NAMED(node_name, "height_offset set to " << height_offset_);      
    pnh_.param("width_offset_mm", width_offset_, 0.0);
      ROS_INFO_STREAM_NAMED(node_name, "width_offset set to " << width_offset_);      

    if(received_alignPubConfig_)
    {
      pnh_.param("x", x_, alignPubConfig_.x);
      pnh_.param("y", y_, alignPubConfig_.y);
      pnh_.param("z", z_, alignPubConfig_.z);
      pnh_.param("roll", roll_, alignPubConfig_.roll);
      pnh_.param("pitch", pitch_, alignPubConfig_.pitch);
      pnh_.param("yaw", yaw_, alignPubConfig_.yaw);
    }
    else
    {
      ROS_INFO_STREAM_NAMED(node_name, "Proceeding without alignment publisher using identify transform for initial guess.");
      pnh_.param("x", x_, 0.0);
      pnh_.param("y", y_, 0.0);
      pnh_.param("z", z_, 0.0);
      pnh_.param("roll", roll_, 0.0);
      pnh_.param("pitch", pitch_, 0.0);
      pnh_.param("yaw", yaw_, 0.0);
    }

      ROS_INFO_STREAM_NAMED(node_name, "x set to " << alignPubConfig_.x);
      ROS_INFO_STREAM_NAMED(node_name, "y set to " << alignPubConfig_.y);
      ROS_INFO_STREAM_NAMED(node_name, "z set to " << alignPubConfig_.z);
      ROS_INFO_STREAM_NAMED(node_name, "roll set to " << alignPubConfig_.roll);
      ROS_INFO_STREAM_NAMED(node_name, "pitch set to " << alignPubConfig_.pitch);
      ROS_INFO_STREAM_NAMED(node_name, "yaw set to " << alignPubConfig_.yaw);

    pnh_.param<std::string>("input_cloud_topic", input_cloud_topic_, "input_cloud");
      ROS_INFO_STREAM_NAMED(node_name, "input_cloud_topic set to " << input_cloud_topic_);
    pnh_.param<std::string>("input_image_topic", input_image_topic_, "input_image");
      ROS_INFO_STREAM_NAMED(node_name, "input_image_topic set to " << input_image_topic_);
    pnh_.param<std::string>("input_info_topic", input_info_topic_, "input_info");
      ROS_INFO_STREAM_NAMED(node_name, "input_info_topic set to " << input_info_topic_);
    pnh_.param<std::string>("image_cloud_topic", image_cloud_topic_, "image_cloud");
      ROS_INFO_STREAM_NAMED(node_name, "image_cloud_topic set to " << image_cloud_topic_);
    pnh_.param<std::string>("filter_cloud_topic", filter_cloud_topic_, "filter_cloud");
      ROS_INFO_STREAM_NAMED(node_name, "filter_cloud_topic set to " << filter_cloud_topic_);
    pnh_.param<std::string>("output_cloud_topic", output_cloud_topic_, "output_cloud");
      ROS_INFO_STREAM_NAMED(node_name, "output_cloud_topic set to " << output_cloud_topic_);
    pnh_.param<std::string>("output_camera_topic", output_camera_topic_, "output_camera");
      ROS_INFO_STREAM_NAMED(node_name, "output_camera_topic set to " << output_camera_topic_);
    pnh_.param<std::string>("output_marker_topic", output_marker_topic_, "output_marker");
      ROS_INFO_STREAM_NAMED(node_name, "output_marker_topic set to " << output_marker_topic_);
    pnh_.param<std::string>("output_tranform", output_trans_topic_, "output_trans");
      ROS_INFO_STREAM_NAMED(node_name, "output_trans_topic set to " << output_trans_topic_);
    pnh_.param("output_frequency", output_frequency_, 10.0);
      ROS_INFO_STREAM_NAMED(node_name, "output_frequency set to " << output_frequency_);     
    pnh_.param("is_rectified", is_rectified_, false);
      ROS_INFO_STREAM_NAMED(node_name, "is_rectified set to " << is_rectified_);     
      
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

    pnh_.param("plane_tol", alignToolConfig_.plane_tol, 0.05);
      ROS_INFO_STREAM_NAMED(node_name, "plane_tol set to " << alignToolConfig_.plane_tol);

  //initialized source classes
    sourceCloud_ = new SourceCloud(nh_, input_cloud_topic_);
    pcl::PCLPointCloud2 cloud; tf2::Transform transform;
    std_msgs::Header header;
    sourceCloud_->cloud_buffer.push_back(cloud);
    sourceCloud_->transform_buffer.push_back(transform);
    sourceCloud_->header_buffer.push_back(header);
    sourceImage_ = new SourceCamera(nh_, input_image_topic_, input_info_topic_);

    drServer_->updateConfig(alignToolConfig_);

  // ROS publishers
    output_trans_pub_  = nh_.advertise<geometry_msgs::TransformStamped>(output_trans_topic_,100);
    image_cloud_pub_   = nh_.advertise<sensor_msgs::PointCloud2>(image_cloud_topic_,10);
    filter_cloud_pub_  = nh_.advertise<sensor_msgs::PointCloud2>(filter_cloud_topic_,10);
    output_cloud_pub_  = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_,10);
    output_camera_pub_ = nh_.advertise<sensor_msgs::Image>(output_camera_topic_,10);
    output_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(output_marker_topic_, 10);
    pub_timer_ = nh_.createTimer(ros::Duration(1.0/output_frequency_), boost::bind(& Chessboard_Alignment::publish_callback, this, _1));

  // ROS subscribers
    sourceCloud_->cloud_sub = nh_.subscribe<sensor_msgs::PointCloud2>(sourceCloud_->cloud_in_topic, 10, boost::bind(&Chessboard_Alignment::input_cloud_callback, this, _1));
    sourceImage_->cameraSync->registerCallback(boost::bind(&Chessboard_Alignment::input_camera_callback, this, _1, _2));

  // ROS Services
    service0_ = pnh_.advertiseService("reset", &Chessboard_Alignment::reset_callback, this);
    service1_ = pnh_.advertiseService("push_transform", &Chessboard_Alignment::pushtransform_callback, this);

  // Reset the guess transform for good measure
    Chessboard_Alignment::reset();

    ROS_INFO_STREAM_NAMED(node_name, node_name.c_str() << " initialized!");
  }

  void Chessboard_Alignment::reconfigure_server_callback(multi_sensor_alignment::cb_align_toolConfig &config, uint32_t level) 
  {
    
    ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %f ", 
            config.plane_tol,
            config.i_min, config.i_max, 
            config.x_min, config.x_max, 
            config.y_min, config.y_max, 
            config.z_min, config.z_max);

    alignToolConfig_ = config;
    received_alignToolConfig_ = true;

    Chessboard_Alignment::reset();
  }

  void Chessboard_Alignment::align_pubconfig_callback(const multi_sensor_alignment::alignment_publisherConfig& config) 
  {
    ROS_INFO("cb_align_tool received new configuration from alignment publisher");
    ROS_INFO("%f %f %f %f %f %f", 
            config.x, config.y, config.z, config.roll, config.pitch, config.yaw);

    x_     = config.x;
    y_     = config.y;
    z_     = config.z;
    roll_  = config.roll;
    pitch_ = config.pitch;
    yaw_   = config.yaw;

    alignPubConfig_ = config;
    received_alignPubConfig_ = true;

    Chessboard_Alignment::reset();
  }

  void Chessboard_Alignment::align_pubdesc_callback(const dynamic_reconfigure::ConfigDescription& description) 
  {
    ROS_INFO("Received description from alignment publisher");

    alignPubDesc_ = description;
    received_alignPubDesc_ = true;
  }

  bool Chessboard_Alignment::pushTransform()
  {
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

  void Chessboard_Alignment::publish_callback(const ros::TimerEvent& event)
  {
  // Gather Input
    // Image
    const std::lock_guard<std::mutex> lock_camera(camera_mutex_);
    if(sourceImage_->image.image.empty()) 
    { 
      ROS_WARN_STREAM("No image found");
      return;
    }
    cv::Mat gray;
    cv::cvtColor(sourceImage_->image.image, gray, CV_BGR2GRAY);

    // Pointcloud
    const std::lock_guard<std::mutex> lock_cloud(cloud_mutex_);
    pcl::PointCloud<PointT>::Ptr lidar_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr image_cloud(new pcl::PointCloud<PointT>);
    if(sourceCloud_->cloud_buffer.back().width < 1 || sourceCloud_->cloud_buffer.back().height < 1)
    {
       ROS_WARN_STREAM("No cloud found");
       return;
    }
    pcl::fromPCLPointCloud2(sourceCloud_->cloud_buffer.back(), *lidar_cloud);
    
  //Check image for Chessboard Pattern
    cv::Size2i patternNum(grid_rows_,grid_cols_);
    std::vector<cv::Point2f> chessCorners;
    bool patternfound = cv::findChessboardCorners(gray, patternNum, chessCorners,
                                                  cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

  //Proceed with transforms if pattern found
    if(patternfound)
    {
       ROS_INFO_STREAM("Chessboard Found");
       
    // Find location of chessboard (image_transform) and create pointcloud (image_cloud) 
       ImageProcessing(gray, chessCorners, image_cloud); 

    //Filter lidar_cloud to area in vicinity of chessboard in the image   
       CloudProcessing(lidar_cloud);


    // Extract output transform as difference between these transforms
       tf2::Transform transform1, transform2;
       
       //image transform
       tf2::convert(image_transform_->transform, transform1);
       tf2::Vector3 vector1(transform1.getOrigin());
       tf2::Matrix3x3 matrix1(transform1.getRotation());
       double x, y, z, roll, pitch, yaw;
       x = vector1.getX();
       y = vector1.getY();
       z = vector1.getZ();
       matrix1.getRPY(roll, pitch, yaw);
         //ROS_INFO_STREAM("image transform");    
         ROS_INFO_STREAM("image_transform->translation\n" << x << "\n" << y << "\n" << z);
         ROS_INFO_STREAM("image_transform->rotation\n"    << roll << "\n" << pitch << "\n" << yaw);
       
       //pointcloud transform
       tf2::convert(cloud_transform_->transform, transform2);
       tf2::Vector3 vector2(transform2.getOrigin());
       tf2::Matrix3x3 matrix2(transform2.getRotation());
       x = vector2.getX();
       y = vector2.getY();
       z = vector2.getZ();
       matrix2.getRPY(roll, pitch, yaw);
         //ROS_INFO_STREAM("cloud transform");
         ROS_INFO_STREAM("cloud_transform->translation\n" << x << "\n" << y << "\n" << z);
         ROS_INFO_STREAM("cloud_transform->rotation\n"    << roll << "\n" << pitch << "\n" << yaw);

       tf2::Transform transform3 = transform1.inverseTimes(transform2);
       tf2::Vector3 vector3(transform3.getOrigin());
       tf2::Matrix3x3 matrix3(transform3.getRotation());
       x = vector3.getX();
       y = vector3.getY();
       z = vector3.getZ();
       matrix3.getRPY(roll, pitch, yaw);
       tf2::convert(transform3, output_transform_->transform); 
         //ROS_INFO_STREAM("alignment transform");
         ROS_INFO_STREAM("align_transform->translation\n" << x << "\n" << y << "\n" << z);
         ROS_INFO_STREAM("align_transform->rotation\n"    << roll << "\n" << pitch << "\n" << yaw);
    }
    else
    {
      ROS_INFO_STREAM("Chessboard Not Found");
    }

  }

  void Chessboard_Alignment::ImageProcessing(cv::Mat &gray, std::vector<cv::Point2f> &chessCorners, pcl::PointCloud<PointT>::Ptr &cloud)
  {
  // Find chessboard features  
    std::vector<cv::Point3f> gridPoints;
    // Find intersecting corner points with sub-pixel accuracy
    cv::cornerSubPix(gray, chessCorners, cv::Size(11,11), cv::Size(-1,-1),
                  cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    cv::Size imgsize;
    imgsize.height = gray.rows;
    imgsize.width = gray.cols;
    cv::Size2i patternNum(grid_rows_,grid_cols_);
    cv::Size2d patternSize(square_size_,square_size_);
    double tx, ty; // Translation values
    // Location of board frame origin from the bottom left inner corner of the chessboard
    tx = (patternNum.height - 1) * patternSize.height/2;
    ty = (patternNum.width - 1) * patternSize.width/2;
    // Board corners w.r.t board frame
    for(int i = 0; i < patternNum.height; i++)
    {
      for(int j = 0; j < patternNum.width; j++)
      {
        cv::Point3f tmp_gridPoint;
        // Translating origin from bottom left corner to the centre of the chessboard
        tmp_gridPoint.x = i*patternSize.height - tx;
        tmp_gridPoint.y = j*patternSize.width - ty;
        tmp_gridPoint.z = 0;
        gridPoints.push_back(tmp_gridPoint);
      }
    }

  // Expected chessboard point coordinates in 3D frame based on origin at the centre of the chess pattern
    // Board corners
    std::vector< cv::Point3f > boardcorners;
    boardcorners.push_back(cv::Point3f( (board_width_ - width_offset_)/2,
                                        (board_height_ - height_offset_)/2, 0.0));
    boardcorners.push_back(cv::Point3f(-(board_width_ + width_offset_)/2,
                                        (board_height_ - height_offset_)/2, 0.0));
    boardcorners.push_back(cv::Point3f(-(board_width_ + width_offset_)/2,
                                        -(board_height_ + height_offset_)/2, 0.0));
    boardcorners.push_back(cv::Point3f( (board_width_ - width_offset_)/2,
                                        -(board_height_ + height_offset_)/2, 0.0));
    // Board center
    boardcorners.push_back(cv::Point3f(-width_offset_/2,
                                        -height_offset_/2, 0.0));

    // // Board centre chessboard square corner coordinates wrt the centre of the board (origin)
    // std::vector< cv::Point3f > square_edge;
    // square_edge.push_back(cv::Point3f(-square_size_/2, -square_size_/2, 0.0));
    // square_edge.push_back(cv::Point3f( square_size_/2,  square_size_/2, 0.0));

    // // chessboard corners, middle square corners, board corners and centre
    // std::vector<cv::Point2f> imagePoints0, imagePoints1, imagePoints2;

  // Find transform using 3D-2D point correspondences.
    cv::Mat rvec(3,3,cv::DataType<double>::type); // Initialization for pinhole and fisheye cameras
    cv::Mat tvec(3,1,cv::DataType<double>::type);
    if (sourceImage_->distortion_model == "plumb_bob")
    {
      //Finds an object pose from 3D-2D point correspondences. 
      //This function returns the rotation and the translation vectors that transform a 3D point expressed in the object coordinate frame to the camera coordinate frame
      cv::solvePnP(gridPoints, chessCorners, sourceImage_->camera_instrinsics, sourceImage_->distortion_coefficients, rvec, tvec);
      // // Convert all to image coordinates 
      // cv::projectPoints(gridPoints, rvec, tvec, sourceImage_->camera_instrinsics, sourceImage_->distortion_coefficients, imagePoints0);
      // cv::projectPoints(square_edge, rvec, tvec, sourceImage_->camera_instrinsics, sourceImage_->distortion_coefficients, imagePoints1);
      // cv::projectPoints(boardcorners, rvec, tvec, sourceImage_->camera_instrinsics, sourceImage_->distortion_coefficients, imagePoints2);
    }
    else
    {
      ROS_WARN_STREAM(sourceImage_->distortion_model << " distortion Model not implemented");
    }
    
  //Convert OpenCv Transform into geometry_msgs::Transform
    Eigen::Affine3d cb_pose;  cb_pose.setIdentity();
    cv::Mat tmprmat = cv::Mat(3,3,CV_64F); // rotation matrix
    cv::Rodrigues(rvec,tmprmat); // rvec to rotation matrix

    for(int j = 0; j < 3; j++)
    {
      for(int k = 0; k < 3; k++)
      {
        cb_pose(j,k) = tmprmat.at<double>(j,k);
      }
      cb_pose(j,3) = tvec.at<double>(j);
    }
    geometry_msgs::Transform transform = tf2::eigenToTransform(cb_pose).transform;
    image_transform_->transform.translation.x = transform.translation.x/1000.0;
    image_transform_->transform.translation.y = transform.translation.y/1000.0;
    image_transform_->transform.translation.z = transform.translation.z/1000.0;
    image_transform_->transform.rotation = transform.rotation;

  //Visualize Points
    // take every point in boardcorners set
    for (int k = 0; k < boardcorners.size(); k++)
    {
      cv::Point3f pt(boardcorners[k]);
      cv::Mat image_points = cv::Mat::eye(3,5,CV_64F);

      // Transform to obtain the coordinates in optical frame
      for (int i = 0; i < 3; i++)
      {
        image_points.at<double>(i,k) = cb_pose(i,0)*pt.x +
            cb_pose(i,1)*pt.y + cb_pose(i,3);
      }

      // convert 3D coordinates to image coordinates
      double * img_coord = Chessboard_Alignment::convert_to_imgpts(image_points.at<double>(0,k),
                                                                   image_points.at<double>(1,k),
                                                                   image_points.at<double>(2,k));
      // Mark the corners and the board centre
      if (k==0)
        cv::circle(sourceImage_->image.image, cv::Point(img_coord[0],img_coord[1]),
            12, CV_RGB(0,255,0),-1); //green
      else if (k==1)
        cv::circle(sourceImage_->image.image, cv::Point(img_coord[0],img_coord[1]),
            12, CV_RGB(255,255,0),-1); //yellow
      else if (k==2)
        cv::circle(sourceImage_->image.image, cv::Point(img_coord[0],img_coord[1]),
            12, CV_RGB(0,0,255),-1); //blue
      else if (k==3)
        cv::circle(sourceImage_->image.image, cv::Point(img_coord[0],img_coord[1]),
            12, CV_RGB(255,0,0),-1); //red
      else
        cv::circle(sourceImage_->image.image, cv::Point(img_coord[0],img_coord[1]),
            12, CV_RGB(255,255,255),-1); //white for centre

      delete[] img_coord;
    }

  // Republish the image with all the features marked on it
    output_camera_pub_.publish(sourceImage_->image.toImageMsg());

  //transform location of chessboard from optical frame to child frame
    tf2::doTransform(*image_transform_, *image_transform_, sourceImage_->transform);

  //Create pointcloud from chessboard location in child frame
    pcl::PointCloud<PointT>::Ptr image_corners(new pcl::PointCloud<PointT>);
    //corners
    for (int k = 0; k < boardcorners.size(); k++)
    {
      PointT temp_point;
      temp_point.x = boardcorners[k].x/1000;
      temp_point.y = boardcorners[k].y/1000;
      temp_point.z = boardcorners[k].z/1000;
      image_corners->points.push_back(temp_point);
    }
    //center
    PointT temp_point;
    temp_point.x = 0.0;
    temp_point.y = 0.0;
    temp_point.z = 0.0;
    image_corners->points.push_back(temp_point);
    //fill
    int n = 20;
    int m = 20;
    for(int i = 0; i < n; i++)
    {
      for(int j = 0; j < m; j++)
      {
        double weight0 = (1.0-i/(double)n) * ((1.0-j/(double)m));
        double weight1 =     (i/(double)n) * ((1.0-j/(double)m));
        double weight2 =     (i/(double)n) *     ((j/(double)m));
        double weight3 = (1.0-i/(double)n) *     ((j/(double)m));
        PointT temp_point;
        temp_point.x = (weight0*boardcorners[0].x + weight1*boardcorners[1].x + weight2*boardcorners[2].x + weight3*boardcorners[3].x)/1000;
        temp_point.y = (weight0*boardcorners[0].y + weight1*boardcorners[1].y + weight2*boardcorners[2].y + weight3*boardcorners[3].y)/1000;
        temp_point.z = (weight0*boardcorners[0].z + weight1*boardcorners[1].z + weight2*boardcorners[2].z + weight3*boardcorners[3].z)/1000;
        // ROS_INFO_STREAM(temp_point.x << ":" << temp_point.y << ":" << temp_point.z);
        cloud->points.push_back(temp_point);
      }
    }

    sensor_msgs::PointCloud2 cloud_pc2; pcl::toROSMsg(*cloud, cloud_pc2);
    tf2::doTransform(cloud_pc2, cloud_pc2, *image_transform_);

    // Publish the cloud
    image_cloud_pub_.publish(cloud_pc2);


  }

  void Chessboard_Alignment::CloudProcessing(pcl::PointCloud<PointT>::Ptr &cloud)
  {
    ROS_INFO_STREAM("Filter Cloud");
  //Filter the cloud using user defined filtering parameters
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    DownsampleCloud(*cloud, *cloud_filtered, 0.0);

  //Further filter the cloud by drawing a cube around the center point found in the image transform
      pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> boxFilter;
    float maxDim = std::max(board_width_, board_height_)/1000;
    float minX = image_transform_->transform.translation.x - 0.55*maxDim;
    float maxX = image_transform_->transform.translation.x + 0.55*maxDim;
    float minY = image_transform_->transform.translation.y - 0.55*maxDim;
    float maxY = image_transform_->transform.translation.y + 0.55*maxDim;
    float minZ = image_transform_->transform.translation.z - 0.55*maxDim;
    float maxZ = image_transform_->transform.translation.z + 0.55*maxDim;
    // ROS_INFO_STREAM("  min:" << minX << ":" << minY << ":" << minZ);
    // ROS_INFO_STREAM("  max:" << maxX << ":" << maxY << ":" << maxZ);
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud_filtered);
    boxFilter.filter(*cloud_filtered2);

    // // Publish the cloud
    sensor_msgs::PointCloud2 cloud_filter;
    pcl::toROSMsg(*cloud_filtered2, cloud_filter);
    filter_cloud_pub_.publish(cloud_filter);

  // //Find Plane of Board
  //   int count = 0;
  //   pcl::PointCloud<PointT>::Ptr cloud_outliers(new pcl::PointCloud<PointT>(*cloud_filtered2));
  //   pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);
  //   pcl::PointCloud<PointT>::Ptr cloud_hull(new pcl::PointCloud<PointT>);
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_corner(new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::PointCloud<PointT>::Ptr cloud_combined(new pcl::PointCloud<PointT>);
  //   cloud_plane->header = cloud_hull->header = cloud_corner->header = cloud_combined->header = cloud_outliers->header;
  //   pcl::PointXYZ point_normal;
    
  //   while(cloud_outliers->size() > 30)
  //   {
  //     count = count + 1;
  //   //Fit plane
  //     ROS_INFO_STREAM("Fit Plane");
  //     //Use image plane as initial guess
  //     pcl::ModelCoefficients::Ptr coefficients_image(new pcl::ModelCoefficients);
  //     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  //     coefficients_image->values.emplace_back(0.0);
  //     coefficients_image->values.emplace_back(0.0);
  //     coefficients_image->values.emplace_back(1.0);
  //     coefficients_image->values.emplace_back(0.0);
  //     ROS_INFO_STREAM("  image plane:" << coefficients_image->values[0] << ":" << coefficients_image->values[1] << ":" << coefficients_image->values[2] << ":" << coefficients_image->values[3]);
  //     coefficients = coefficients_image;

  //     Eigen::Affine3d eigenTransform = tf2::transformToEigen(image_transform_->transform);

  //     // Eigen::Matrix3d m = eigenTransform.rotation();
  //     // Eigen::Vector3d v = eigenTransform.translation();
  //     // Eigen::Quaterniond q = (Eigen::Quaterniond)eigenTransform.linear();
  //     // double x, y, z, qx, qy, qz, qw;
  //     //  x = v.x();
  //     //  y = v.y();
  //     //  z = v.z();
  //     //  qx = q.x();
  //     //  qy = q.y();
  //     //  qz = q.z();
  //     //  qw = q.w();
  //     //    //ROS_INFO_STREAM("image transform");    
  //     // ROS_INFO_STREAM("image_transform->translation\n" << x << "\n" << y << "\n" << z);
  //     // ROS_INFO_STREAM("image_transform->quaterion\n"  << qx << "\n" << qy << "\n" << qz << "\n" << qw);

  //     ROS_INFO_STREAM("  Transform");
  //     pcl::transformPlane(coefficients_image, coefficients, eigenTransform);
  //     ROS_INFO_STREAM("    guess plane:" << coefficients->values[0] << ":" << coefficients->values[1] << ":" << coefficients->values[2] << ":" << coefficients->values[3]);

  //     pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  //     pcl::SACSegmentation<PointT> seg;
  //     seg.setOptimizeCoefficients(true);
  //     seg.setModelType(pcl::SACMODEL_PLANE);
  //     seg.setMethodType(pcl::SAC_RANSAC);
  //     seg.setDistanceThreshold(alignToolConfig_.plane_tol);
  //     seg.setMaxIterations(100);
  //     seg.setInputCloud(cloud_outliers);
  //     seg.segment(*inliers, *coefficients);

  //     if(inliers->indices.size() < 30)
  //     {
  //       ROS_WARN_STREAM("Unable to find plane in cloud segment " << count);
  //       break;
  //     }
  //     else
  //     {
  //        ROS_INFO_STREAM("  fit points:" << inliers->indices.size() << "/" << cloud_outliers->size());
  //        ROS_INFO_STREAM("  fit plane:" << coefficients->values[0] << ":" << coefficients->values[1] << ":" << coefficients->values[2] << ":" << coefficients->values[3]);
  //     }

  //   // Project the inliers on the plane
  //     pcl::ProjectInliers<PointT> proj;
  //     proj.setModelType(pcl::SACMODEL_PLANE);
  //     proj.setInputCloud(cloud_outliers);
  //     proj.setModelCoefficients(coefficients);
  //     proj.filter(*cloud_plane);
  //     float mag = sqrt(pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2)
  //       + pow(coefficients->values[2], 2));
  //     float sign = 1;
  //     // if(coefficients->values[0] > 0) sign = -1;  //Vector should always point back toward camera 
  //     point_normal.x = sign*coefficients->values[0]/mag;
  //     point_normal.y = sign*coefficients->values[1]/mag;
  //     point_normal.z = sign*coefficients->values[2]/mag;
  //     ROS_INFO_STREAM("Plane Points: " << cloud_plane->size());

  //     // Publish the cloud
  //     sensor_msgs::PointCloud2 cloud_final;
  //     pcl::toROSMsg(*cloud_plane, cloud_final);
  //     output_cloud_pub_.publish(cloud_final);


  //     // create a visualization object
  //     // pcl::PointCloud<pcl::PointXYZ>::Ptr view_points (new pcl::PointCloud<pcl::PointXYZ>);
  //     // pcl::copyPointCloud<PointT>(*cloud_plane, *view_points);

  //     // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  //     //  viewer = Multi_Sensor_Alignment::simpleVis(view_points);
  //     //  viewer->spin();


  //     //Extract the outliers for next pass (i.e. current plane does not work out to be the board)
  //     ROS_DEBUG_STREAM("Extract outliers");
  //     pcl::ExtractIndices<PointT> extract;
  //     extract.setInputCloud(cloud_outliers);
  //     extract.setIndices(inliers);
  //     extract.setNegative (true);
  //     extract.filter(*cloud_outliers);

  //   //Use Convex hull to find edges points of plane
  //     ROS_DEBUG_STREAM("Hull");
  //     pcl::ConcaveHull<PointT> chull;
  //     chull.setAlpha(board_height_/1000.0/1.0);
  //     chull.setInputCloud(cloud_plane);
  //     chull.reconstruct (*cloud_hull);
  //     if(cloud_hull->points.size() < 15)
  //     {
  //       ROS_WARN_STREAM("Unable to find sufficient edge points on plane " << count);
  //       continue;
  //     }
  //     ROS_DEBUG_STREAM("Hull Points: " << cloud_hull->size());

  //     //Find key points (those with greatest y/z values). These should be the closest points to the corners. 
  //     // ToDo there is an underlying assumption that the working frame is X-forward and Z-up, really need a more robust solution
  //     //Start and bottom and go clockwise to organize the points
  //     ROS_DEBUG_STREAM("Key Points");
  //     pcl::PointCloud<PointT> key_points;
  //     for(int i = 0; i < 4; i++) key_points.push_back(cloud_hull->points[0]);
  //     for(int i = 0; i < cloud_hull->size(); i++)
  //     {
  //       if(cloud_hull->points[i].z < key_points.points[0].z) key_points.points[0] = cloud_hull->points[i];
  //       if(cloud_hull->points[i].y < key_points.points[1].y) key_points.points[1] = cloud_hull->points[i];
  //       if(cloud_hull->points[i].z > key_points.points[2].z) key_points.points[2] = cloud_hull->points[i];
  //       if(cloud_hull->points[i].y > key_points.points[3].y) key_points.points[3] = cloud_hull->points[i];
  //     }
  //     ROS_DEBUG_STREAM("Key Points: " << key_points.points[0] << key_points.points[1] << key_points.points[2] << key_points.points[3]);

  //     //Sort into edges using key points as boundaries
  //     ROS_DEBUG_STREAM("Group Edges");
  //     pcl::PointCloud<PointT>::Ptr downright(new pcl::PointCloud<PointT>);
  //     pcl::PointCloud<PointT>::Ptr upright(new pcl::PointCloud<PointT>);
  //     pcl::PointCloud<PointT>::Ptr upleft(new pcl::PointCloud<PointT>);
  //     pcl::PointCloud<PointT>::Ptr downleft(new pcl::PointCloud<PointT>);
  //     for(int i = 0; i < cloud_hull->size(); i++)
  //     {
  //       double plane_tol = alignToolConfig_.plane_tol;
  //       if( (cloud_hull->points[i].y < (key_points.points[0].y))            && (cloud_hull->points[i].z < (key_points.points[1].z)) 
  //        && (cloud_hull->points[i].y > (key_points.points[1].y+ plane_tol)) && (cloud_hull->points[i].z > (key_points.points[0].z + plane_tol))) { downright->push_back(cloud_hull->points[i]); cloud_combined->push_back(cloud_hull->points[i]); }
  //       if( (cloud_hull->points[i].y < (key_points.points[2].y))            && (cloud_hull->points[i].z > (key_points.points[1].z)) 
  //        && (cloud_hull->points[i].y > (key_points.points[1].y+ plane_tol)) && (cloud_hull->points[i].z < (key_points.points[2].z - plane_tol))) { upright->push_back(cloud_hull->points[i]); cloud_combined->push_back(cloud_hull->points[i]); }
  //       if( (cloud_hull->points[i].y > (key_points.points[2].y))            && (cloud_hull->points[i].z > (key_points.points[3].z)) 
  //        && (cloud_hull->points[i].y < (key_points.points[3].y- plane_tol)) && (cloud_hull->points[i].z < (key_points.points[2].z - plane_tol))) { upleft->push_back(cloud_hull->points[i]); cloud_combined->push_back(cloud_hull->points[i]); }
  //       if( (cloud_hull->points[i].y > (key_points.points[0].y))            && (cloud_hull->points[i].z < (key_points.points[3].z)) 
  //        && (cloud_hull->points[i].y < (key_points.points[3].y- plane_tol)) && (cloud_hull->points[i].z > (key_points.points[0].z + plane_tol))) { downleft->push_back(cloud_hull->points[i]); cloud_combined->push_back(cloud_hull->points[i]); }
  //     }
  //     ROS_INFO_STREAM("Edge Points: " << downright->points.size() << "," << upright->points.size() << "," << upleft->points.size() << "," << downleft->points.size());
  //     if((downright->points.size() < 3) || (upright->points.size() < 3) || (upleft->points.size() < 3) || (downleft->points.size() < 3))
  //     {
  //       ROS_WARN_STREAM("Insufficient points along one edge of plane " << count);
  //       continue;
  //     }

  //     //Fit lines through the the edge points
  //     ROS_DEBUG_STREAM("Fit Lines");
  //     std::vector<pcl::ModelCoefficients::Ptr> l_coefficients;
  //     std::vector<pcl::PointIndices::Ptr> l_inliers;
  //     pcl::PointCloud<PointT>::Ptr hull_outliers(new pcl::PointCloud<PointT>(*cloud_hull));

  //     seg.setModelType(pcl::SACMODEL_LINE);
  //     seg.setMethodType(pcl::SAC_RANSAC);
  //     seg.setDistanceThreshold(alignToolConfig_.plane_tol*2);
  //     seg.setOptimizeCoefficients(true);
      
  //     pcl::ModelCoefficients::Ptr a_coefficients (new pcl::ModelCoefficients);
  //     pcl::PointIndices::Ptr a_inliers (new pcl::PointIndices);
  //     seg.setInputCloud(downright);
  //     seg.segment (*a_inliers, *a_coefficients);
  //     l_coefficients.push_back(a_coefficients);
  //     //ROS_DEBUG_STREAM("downright: " << a_coefficients->values.size());

      
  //     pcl::ModelCoefficients::Ptr b_coefficients (new pcl::ModelCoefficients);
  //     pcl::PointIndices::Ptr b_inliers (new pcl::PointIndices);
  //     seg.setInputCloud(upright);
  //     seg.segment (*b_inliers, *b_coefficients);
  //     l_coefficients.push_back(b_coefficients);
  //     //ROS_DEBUG_STREAM("upright: " << b_coefficients->values.size());

  //     pcl::ModelCoefficients::Ptr c_coefficients (new pcl::ModelCoefficients);
  //     pcl::PointIndices::Ptr c_inliers (new pcl::PointIndices);
  //     seg.setInputCloud(upleft);
  //     seg.segment (*c_inliers, *c_coefficients);
  //     l_coefficients.push_back(c_coefficients);
  //     //ROS_DEBUG_STREAM("upleft: " << c_coefficients->values.size());

  //     pcl::ModelCoefficients::Ptr d_coefficients (new pcl::ModelCoefficients);
  //     pcl::PointIndices::Ptr d_inliers (new pcl::PointIndices);
  //     seg.setInputCloud(downleft);  // might need to clear inliers and coefficients
  //     seg.segment (*d_inliers, *d_coefficients);
  //     l_coefficients.push_back(d_coefficients);
  //     //ROS_DEBUG_STREAM("downleft: " << d_coefficients->values.size());

  //   //Find board corners as line intersections
  //     ROS_DEBUG_STREAM("Find Intersections");
  //     Eigen::Vector4f Point_l;
  //     pcl::PointXYZ basic_point; // intersection points stored here
  //     double plane_tol = alignToolConfig_.plane_tol;
  //     if(pcl::lineWithLineIntersection(*l_coefficients[3], *l_coefficients[0], Point_l, plane_tol*plane_tol/4.))
  //     {
  //       basic_point.x = Point_l[0];
  //       basic_point.y = Point_l[1];
  //       basic_point.z = Point_l[2];
  //       cloud_corner->points.push_back(basic_point);
  //       ROS_DEBUG_STREAM("point 1=" << basic_point);
  //     }
  //     else
  //     {
  //       ROS_WARN_STREAM("Unable to find corner point 1 in plane " << count);
  //       continue;
  //     }
      
  //     if(pcl::lineWithLineIntersection(*l_coefficients[0], *l_coefficients[1], Point_l, plane_tol*plane_tol/4.))
  //     {
  //       basic_point.x = Point_l[0];
  //       basic_point.y = Point_l[1];
  //       basic_point.z = Point_l[2];
  //       cloud_corner->points.push_back(basic_point);
  //       ROS_DEBUG_STREAM("point 2=" << basic_point);
  //     }
  //           else
  //     {
  //       ROS_WARN_STREAM("Unable to find corner point 2 in plane " << count);
  //       continue;
  //     }

  //     if(pcl::lineWithLineIntersection(*l_coefficients[1], *l_coefficients[2], Point_l, plane_tol*plane_tol/4.))
  //     {
  //       basic_point.x = Point_l[0];
  //       basic_point.y = Point_l[1];
  //       basic_point.z = Point_l[2];
  //       cloud_corner->points.push_back(basic_point);
  //       ROS_DEBUG_STREAM("point 3=" << basic_point);
  //     }
  //     else
  //     {
  //       ROS_WARN_STREAM("Unable to find corner point 3 in plane " << count);
  //       continue;
  //     }
  //     if(pcl::lineWithLineIntersection(*l_coefficients[2], *l_coefficients[3], Point_l, plane_tol*plane_tol/4.))
  //     {
  //       basic_point.x = Point_l[0];
  //       basic_point.y = Point_l[1];
  //       basic_point.z = Point_l[2];
  //       cloud_corner->points.push_back(basic_point);
  //       ROS_DEBUG_STREAM("point 4=" << basic_point);
  //     }
  //     else
  //     {
  //       ROS_WARN_STREAM("Unable to find corner point 4 in plane " << count);
  //       continue;
  //     }

  //     // Check length of diagonals
  //     float diagonal1x = cloud_corner->points[2].x-cloud_corner->points[0].x;
  //     float diagonal1y = cloud_corner->points[2].y-cloud_corner->points[0].y;
  //     float diagonal1z = cloud_corner->points[2].z-cloud_corner->points[0].z;
  //     float diagonal2x = cloud_corner->points[3].x-cloud_corner->points[1].x;
  //     float diagonal2y = cloud_corner->points[3].y-cloud_corner->points[1].y;
  //     float diagonal2z = cloud_corner->points[3].z-cloud_corner->points[1].z;
  //     float actual_length1 = diagonal1x*diagonal1x + diagonal1y*diagonal1y + diagonal1z*diagonal1z;
  //     float actual_length2 = diagonal2x*diagonal2x + diagonal2y*diagonal2y + diagonal2z*diagonal2z;
  //     float expected_length = board_width_*board_width_ + board_height_*board_height_;
  //     if(actual_length1 < 0.9*expected_length || actual_length1 > 1.1*expected_length  ||
  //        actual_length2 < 0.9*expected_length || actual_length2 > 1.1*expected_length)
  //     {
  //       ROS_WARN_STREAM("Board dimensions do not work in plane " << count);
  //       continue;
  //     } 
      
  //     //Find center from corner diagonals
  //     pcl::PointXYZ center1;
  //     center1.x = (cloud_corner->points[2].x+cloud_corner->points[0].x)/2.0;
  //     center1.y = (cloud_corner->points[2].y+cloud_corner->points[0].y)/2.0;
  //     center1.z = (cloud_corner->points[2].z+cloud_corner->points[0].z)/2.0;
  //     pcl::PointXYZ center2;
  //     center2.x = (cloud_corner->points[3].x+cloud_corner->points[1].x)/2.0;
  //     center2.y = (cloud_corner->points[3].y+cloud_corner->points[1].y)/2.0;
  //     center2.z = (cloud_corner->points[3].z+cloud_corner->points[1].z)/2.0;
  //     if(((center1.x - center2.x)*(center1.x - center2.x) +
  //         (center1.y - center2.y)*(center1.y - center2.y) + 
  //         (center1.z - center2.z)*(center1.z - center2.z)) > plane_tol*plane_tol);
  //     {
  //       ROS_WARN_STREAM("Board diagonals do not cross on plane " << count);
  //       continue;
  //     } 

  //     basic_point.x = (center1.x + center2.x)/2.0;
  //     basic_point.y = (center1.y + center2.y)/2.0;
  //     basic_point.z = (center1.z + center2.z)/2.0;
  //     cloud_corner->points.push_back(basic_point);
  //     ROS_DEBUG_STREAM("point 5=" << basic_point);
      
  //     break;
  //   }
  
  // // Publish the cloud
  //   sensor_msgs::PointCloud2 cloud_final;
  //   if(cloud_corner->points.size() == 5) pcl::toROSMsg(*cloud_combined, cloud_final);
  //   else pcl::toROSMsg(*cloud_hull, cloud_final);
  //   ROS_DEBUG_STREAM("Publishing Cloud");
  //   output_cloud_pub_.publish(cloud_final);

  //   if(cloud_corner->points.size() != 5) return;

  // //Turn results into a pose
  //   Eigen::Vector3d xy_normal_vector, board_normal_vector, plane_rotation_vector;
  //   xy_normal_vector[0] = 0.0;
  //   xy_normal_vector[1] = 0.0;
  //   xy_normal_vector[2] = 1.0;
  //   board_normal_vector[0] = point_normal.x;
  //   board_normal_vector[1] = point_normal.y;
  //   board_normal_vector[2] = point_normal.z;
  //   board_normal_vector.normalize();
  //   plane_rotation_vector = xy_normal_vector.cross(board_normal_vector);
  //   double plane_rotation_angle = atan2(plane_rotation_vector.norm(), xy_normal_vector.dot(board_normal_vector));
  //   plane_rotation_vector.normalize();
 
  //   Eigen::Affine3d cb_pose = Eigen::Affine3d::Identity();
  //   cb_pose = Eigen::AngleAxisd(plane_rotation_angle, plane_rotation_vector);
  //   cb_pose(0,3) = cloud_corner->points[4].x;
  //   cb_pose(1,3) = cloud_corner->points[4].y;
  //   cb_pose(2,3) = cloud_corner->points[4].z;
  //   //ROS_INFO_STREAM("corners camera frame" << "\n" << cloud_corner->points[0] << "\n" << cloud_corner->points[1] << "\n" << cloud_corner->points[2] << "\n" << cloud_corner->points[3] << "\n" << cloud_corner->points[4]);

  // // The transform, cb_pose, as it stands will transform into the plane of the board.  
  //   // However, we are still lacking the final orientation about the normal of this plane.
  //   // We need another rotation transform to align the board edges with the xy axis of the board coordinate system.
  //   geometry_msgs::TransformStamped inverse = tf2::eigenToTransform(cb_pose);
  //   tf2::Transform transform_tf; tf2::convert(inverse.transform, transform_tf);
  //   tf2::Transform inverse_tf = transform_tf.inverse();
  //   tf2::convert(inverse_tf, inverse.transform);
  //   sensor_msgs::PointCloud2 corners_xy; pcl::toROSMsg(*cloud_corner, corners_xy);
  //   tf2::doTransform(corners_xy, corners_xy, inverse);
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_corner_xy(new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::fromROSMsg(corners_xy, *cloud_corner_xy);
    
  //   //ROS_INFO_STREAM("corners board unoriented" << "\n" << cloud_corner_xy->points[0] << "\n" << cloud_corner_xy->points[1] << "\n" << cloud_corner_xy->points[2] << "\n" << cloud_corner_xy->points[3] << "\n" << cloud_corner_xy->points[4]);
    
  //   // Additional rotation vector
  //   Eigen::Vector3d x_vector, orient_rotation;
  //   x_vector[0] = 1.0;
  //   x_vector[1] = 0.0;
  //   x_vector[2] = 0.0;
  //   Eigen::Vector4d rotation_angles(0.0,0.0,0.0,0.0);
  //   //ROS_INFO_STREAM("corner angles");
  //   // calculate rotations for all edges and take average
  //   cloud_corner_xy->points[4] = cloud_corner_xy->points[0]; //copy first value to end of list for easy loopback.
  //   for(int i = 0; i < 4; i++)
  //   {
  //     Eigen::Vector3d edge_vector, rotation_vector;
  //     edge_vector[0] = std::abs(cloud_corner_xy->points[i+1].x - cloud_corner_xy->points[i].x);
  //     edge_vector[1] = std::abs(cloud_corner_xy->points[i+1].y - cloud_corner_xy->points[i].y);
  //     edge_vector[2] = std::abs(cloud_corner_xy->points[i+1].z - cloud_corner_xy->points[i].z);
  //     rotation_vector = x_vector.cross(edge_vector);
  //     rotation_angles[i] = atan2(rotation_vector.norm(), x_vector.dot(edge_vector));
  //     if(rotation_angles[i] > +PI/4) rotation_angles[i] = +PI/2 - rotation_angles[i];
  //     if(rotation_angles[i] < -PI/4) rotation_angles[i] = -PI/2 - rotation_angles[i];
  //     // ROS_INFO_STREAM(rotation_angles[i]*180.0/PI);
  //   }
  //    //Update Transform
  //   cb_pose = Eigen::AngleAxisd(PI/2-rotation_angles.mean(), board_normal_vector)
  //            *Eigen::AngleAxisd(plane_rotation_angle, plane_rotation_vector);
  //   cb_pose(0,3) = cloud_corner->points[4].x;
  //   cb_pose(1,3) = cloud_corner->points[4].y;
  //   cb_pose(2,3) = cloud_corner->points[4].z;
    
  // // Check transform by applying the inverse transform to the points.  
  //   // Should end up with all points in x,y plane and symmetric about 0,0.
  //   geometry_msgs::TransformStamped inverse2 = tf2::eigenToTransform(cb_pose);
  //   tf2::Transform transform_tf2; tf2::convert(inverse2.transform, transform_tf2);
  //   tf2::Transform inverse_tf2 = transform_tf2.inverse();
  //   tf2::convert(inverse_tf2, inverse2.transform);
  //   pcl::toROSMsg(*cloud_corner, corners_xy);
  //   tf2::doTransform(corners_xy, corners_xy, inverse2);
  //   pcl::fromROSMsg(corners_xy, *cloud_corner_xy);
  //   //ROS_INFO_STREAM("corners board frame" << "\n" << cloud_corner_xy->points[0] << "\n" << cloud_corner_xy->points[1] << "\n" << cloud_corner_xy->points[2] << "\n" << cloud_corner_xy->points[3] << "\n" << cloud_corner_xy->points[4]);
    
  //   geometry_msgs::TransformStamped transform = tf2::eigenToTransform(cb_pose);
  //   cloud_transform_->transform.translation.x = transform.transform.translation.x;
  //   cloud_transform_->transform.translation.y = transform.transform.translation.y;
  //   cloud_transform_->transform.translation.z = transform.transform.translation.z;
  //   cloud_transform_->transform.rotation = transform.transform.rotation;    

  //   //Visualize the corner points of velodyne board, 4 corners and center
  //   visualization_msgs::Marker corners_board;
  //   corners_board.header.frame_id = cloud->header.frame_id;
  //   corners_board.header.stamp = pcl_conversions::fromPCL(cloud->header.stamp);
  //   corners_board.ns = "my_sphere";
  //   corners_board.type = visualization_msgs::Marker::SPHERE;
  //   corners_board.action = visualization_msgs::Marker::ADD;
  //   corners_board.pose.orientation.w = 1.0;
  //   corners_board.scale.x = 0.1; corners_board.scale.y = 0.1; corners_board.scale.z = 0.1;
  //   corners_board.color.a = 1.0;
  //   for (int i = 0; i < cloud_corner->points.size(); i++)
  //   {
  //     corners_board.pose.position.x = cloud_corner->points[i].x;
  //     corners_board.pose.position.y = cloud_corner->points[i].y;
  //     corners_board.pose.position.z = cloud_corner->points[i].z;
      
  //     corners_board.id = i;
  //     if (corners_board.id == 0) {
  //       corners_board.color.r = 1.0; corners_board.color.g = 1.0; corners_board.color.b = 0.0; }
  //     else if (corners_board.id == 1) {
  //       corners_board.color.r = 0.0; corners_board.color.g = 0.0; corners_board.color.b = 1.0; }
  //     else if (corners_board.id == 2) {
  //       corners_board.color.r = 1.0; corners_board.color.b = 0.0; corners_board.color.g = 0.0; }
  //     else if (corners_board.id == 3) {
  //       corners_board.color.r = 0.0; corners_board.color.g = 1.0; corners_board.color.b = 0.0;  }
  //     else if (corners_board.id == 4) {
  //       corners_board.color.r = 1.0; corners_board.color.g = 1.0; corners_board.color.b = 1.0;  }
  //     output_marker_pub_.publish(corners_board);
  //   }

  //   // Visualize board normal vector
  //   visualization_msgs::Marker normal;
  //   normal.header.frame_id = cloud->header.frame_id;
  //   normal.header.stamp = pcl_conversions::fromPCL(cloud->header.stamp);
  //   normal.ns = "my_normal";
  //   normal.id = 12;
  //   normal.type = visualization_msgs::Marker::ARROW;
  //   normal.action = visualization_msgs::Marker::ADD;
  //   normal.scale.x = 0.04; normal.scale.y = 0.06; normal.scale.z = 0.1;
  //   normal.color.a = 1.0; normal.color.r = 0.0; normal.color.g = 0.0; normal.color.b = 1.0;
  //   geometry_msgs::Point start, end;
  //   start.x = cloud_corner->points[4].x;
  //   start.y = cloud_corner->points[4].y;
  //   start.z = cloud_corner->points[4].z;
  //   end.x = start.x + point_normal.x/2;
  //   end.y = start.y + point_normal.y/2;
  //   end.z = start.z + point_normal.z/2;
  //   normal.points.resize(2);
  //   normal.points[0].x = start.x;
  //   normal.points[0].y = start.y;
  //   normal.points[0].z = start.z;
  //   normal.points[1].x = end.x;
  //   normal.points[1].y = end.y;
  //   normal.points[1].z = end.z;
  //   output_marker_pub_.publish(normal);

  }
  // /// Sorts lines by how horizontal they are in the xz plane.  Used to match near horizontal lines to near vertical lines for finding corners by line intersection.
  // bool Chessboard_Alignment::compareLineCoeff(const pcl::ModelCoefficients::Ptr &c1, const pcl::ModelCoefficients::Ptr &c2)
  // { 
  //   double x1 = c1->values[3], x2 = c2->values[3];
  //   double y1 = c1->values[5], y2 = c2->values[5];
  //   if( y1 < 0.0) x1 *= -1;
  //   if( y2 < 0.0) x2 *= -1;
  //   return (x1 > x2);
  // }
  // /// Sorts points by z value.
  // bool Chessboard_Alignment::comparePoints(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2)
  // { 
  //   return (p1.z < p2.z);
  // } 
  
  void Chessboard_Alignment::input_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    const std::lock_guard<std::mutex> lock(cloud_mutex_);
    //Transform to storage coordinates
    sensor_msgs::PointCloud2 cloud_in(*msg), cloud_transformed;
    geometry_msgs::TransformStamped transform;
    
    if(parent_frame_id_ != "")
    {
      try
      {
        transform = tfBuffer_.lookupTransform(parent_frame_id_, msg->header.frame_id, msg->header.stamp);
        tf2::doTransform(*msg, cloud_transformed, transform);
      }
      catch (tf2::TransformException ex)
      {
        ROS_WARN_STREAM_NAMED(node_name, "Cloud Callback: " << ex.what());
        return;
      }
    }

    pcl::PCLPointCloud2 pcl_out;
    pcl_conversions::toPCL(cloud_transformed, pcl_out);
    
    sourceCloud_->cloud_buffer.back() = pcl_out;
    tf2::Transform currentPose;
    tf2::convert(transform.transform, currentPose);
    sourceCloud_->transform_buffer.back() = currentPose;
    sourceCloud_->header_buffer.back() = msg->header;

    return;
  }

  void Chessboard_Alignment::input_camera_callback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    const std::lock_guard<std::mutex> myLock(camera_mutex_);
    //Obtain info for transform
    sourceImage_->frameID = image_msg->header.frame_id;
    geometry_msgs::TransformStamped transform;

    if(child_frame_id_ != "")
    {
      try
      {
        transform = tfBuffer_.lookupTransform(child_frame_id_, image_msg->header.frame_id, image_msg->header.stamp);
        sourceImage_->transform = transform;
      }
      catch (tf2::TransformException ex)
      {
        ROS_WARN_STREAM_NAMED(node_name, "Camera Callback: " << ex.what());
        return;
      }
    }

    //Extract camera_info
    std::string distortion_model = info_msg->distortion_model;

    cv::Mat camera_instrinsics = cv::Mat(3, 3, CV_64F);
    for (int row = 0; row < 3; row++)
    {
      for (int col = 0; col < 3; col++)
      {
        camera_instrinsics.at<double>(row, col) = info_msg->K[row * 3 + col];
      }
    }

    cv::Mat distortion_coefficients = cv::Mat(1, 5, CV_64F);
    for (int col = 0; col < 5; col++)
    {
      if(is_rectified_)
        distortion_coefficients.at<double>(col) = 0.0;
      else
        distortion_coefficients.at<double>(col) = info_msg->D[col];
    }

    cv::Mat projection_matrix = cv::Mat(3, 4, CV_64F);
    for (int row = 0; row < 3; row++)
    {
      for (int col = 0; col < 4; col++)
      {
        projection_matrix.at<double>(row, col) = info_msg->P[row * 4 + col];
        // if(is_rectified_ && col < 3)
        //     camera_instrinsics.at<double>(row, col) = info_msg->P[row * 4 + col];
      }
    }

    //Save to storage class
    sourceImage_->distortion_model = distortion_model; //ROS_INFO_STREAM(distortion_model);
    sourceImage_->camera_instrinsics = camera_instrinsics;  //ROS_INFO_STREAM(camera_instrinsics);
    sourceImage_->distortion_coefficients = distortion_coefficients; //ROS_INFO_STREAM(distortion_coefficients);
    sourceImage_->projection_matrix = projection_matrix; //ROS_INFO_STREAM(projection_matrix);

    sourceImage_->fx = static_cast<float>(info_msg->P[0]);
    sourceImage_->fy = static_cast<float>(info_msg->P[5]);
    sourceImage_->cx = static_cast<float>(info_msg->P[2]);
    sourceImage_->cy = static_cast<float>(info_msg->P[6]);

    //Extract image
    try
    {
      cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
      sourceImage_->image = *cv_image;
            
      int count = sourceImage_->image.image.rows * sourceImage_->image.image.cols;
      ROS_DEBUG_STREAM_NAMED(node_name, "Image encoding " << image_msg->encoding);
      ROS_DEBUG_STREAM_NAMED(node_name, sourceImage_->cameraImage_topic  << " has " << count << " pixels.");
    }
    catch (cv_bridge::Exception& e) 
    {
      ROS_ERROR("cv_bridge exception: %s", e.what()); 
    }
  }

  bool Chessboard_Alignment::reset()
  {
    //Reset Guess
    geometry_msgs::Vector3 v;
    v.x = x_; v.y = y_; v.z = z_;

    tf2::Quaternion tf_q; geometry_msgs::Quaternion q;
    tf_q.setRPY(roll_, pitch_, yaw_);
    tf2::convert(tf_q, q);

    geometry_msgs::Transform last_transform;
    last_transform.translation = v;
    last_transform.rotation = q;
    
    Eigen::Affine3d eigenTransform = tf2::transformToEigen(last_transform);
    //current_guess_ = eigenTransform.matrix().cast<float>();
    
    //Reset rolling window accumulators
    x_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    y_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    z_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qx_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qy_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qz_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    qw_array_ = window_acc(tag::rolling_window::window_size = buffer_size_);
    current_qx_ = 0; current_qy_ = 0; current_qz_ = 0; current_qw_ = 0; 
    
    ROS_DEBUG_STREAM_NAMED(node_name, "Guess transform reset");

    return true;
  }

  bool Chessboard_Alignment::reset_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Chessboard_Alignment::reset();
  }

  bool Chessboard_Alignment::pushtransform_callback(std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp)
  {
    return Chessboard_Alignment::pushTransform();
  }


  void Chessboard_Alignment::DownsampleCloud(const pcl::PointCloud<PointT> &in_cloud,
                                                pcl::PointCloud<PointT> &out_cloud,
                                                double in_leaf_size)
  {
    pcl::PointCloud<PointT>::Ptr filtered_ptr(new pcl::PointCloud<PointT>);
    filtered_ptr->clear();
    filtered_ptr->header = in_cloud.header;
    for (int i = 0; i < in_cloud.points.size(); ++i)
    {
      if(in_cloud.points[i].intensity >= alignToolConfig_.i_min && in_cloud.points[i].intensity <= alignToolConfig_.i_max)
      if(in_cloud.points[i].x         >= alignToolConfig_.x_min && in_cloud.points[i].x <= alignToolConfig_.x_max)
      if(in_cloud.points[i].y         >= alignToolConfig_.y_min && in_cloud.points[i].y <= alignToolConfig_.y_max)
      if(in_cloud.points[i].z         >= alignToolConfig_.z_min && in_cloud.points[i].z <= alignToolConfig_.z_max)
        filtered_ptr->push_back(in_cloud.points[i]);
    }

    if(in_leaf_size > 0.001)
    {
      pcl::VoxelGrid<PointT> voxelized;
      voxelized.setInputCloud(filtered_ptr);
      voxelized.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
      voxelized.filter(out_cloud);
    }
    else
    {
      pcl::copyPointCloud(*filtered_ptr, out_cloud);
    }
  }

  // Convert 3D points w.r.t camera frame to 2D pixel points in image frame
  double * Chessboard_Alignment::convert_to_imgpts(double x, double y, double z)
  {  
    double tmpxC = x/z;
    double tmpyC = y/z;
    cv::Point2d planepointsC;
    planepointsC.x = tmpxC;
    planepointsC.y = tmpyC;
    double r2 = tmpxC*tmpxC + tmpyC*tmpyC;

    if (sourceImage_->distortion_model == "plumb_bob")
    {
      double tmpdist = 1 + sourceImage_->distortion_coefficients.at<double>(0)*r2 + sourceImage_->distortion_coefficients.at<double>(1)*r2*r2 +
          sourceImage_->distortion_coefficients.at<double>(4)*r2*r2*r2;
      planepointsC.x = tmpxC*tmpdist + 2*sourceImage_->distortion_coefficients.at<double>(2)*tmpxC*tmpyC +
          sourceImage_->distortion_coefficients.at<double>(3)*(r2+2*tmpxC*tmpxC);
      planepointsC.y = tmpyC*tmpdist + sourceImage_->distortion_coefficients.at<double>(2)*(r2+2*tmpyC*tmpyC) +
          2*sourceImage_->distortion_coefficients.at<double>(3)*tmpxC*tmpyC;
      planepointsC.x = sourceImage_->camera_instrinsics.at<double>(0,0)*planepointsC.x + sourceImage_->camera_instrinsics.at<double>(0,2);
      planepointsC.y = sourceImage_->camera_instrinsics.at<double>(1,1)*planepointsC.y + sourceImage_->camera_instrinsics.at<double>(1,2);
    }

    double * img_coord = new double[2];
    *(img_coord) = planepointsC.x;
    *(img_coord+1) = planepointsC.y;

    return img_coord;
  }  

}  // namespace Multi_Sensor_Alignment





