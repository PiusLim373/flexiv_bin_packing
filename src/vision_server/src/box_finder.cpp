#include "vision_server/box_finder.hpp"
BoxFinder::BoxFinder() : Node("box_finder_node")
{
  this->declare_parameter<double>("max_pass_filter", 0.7);
  this->declare_parameter<double>("voxel_grid_size", 0.01);
  this->declare_parameter<double>("plane_distance_threshold", 0.01);
  this->declare_parameter<int>("min_cluster_size", 150);
  this->declare_parameter<double>("max_pass_filter_fm", 0.5);
  this->declare_parameter<double>("voxel_grid_size_fm", 0.01);
  this->declare_parameter<int>("min_cluster_size_fm", 50);

  this->get_parameter("max_pass_filter", max_pass_filter);
  this->get_parameter("voxel_grid_size", voxel_grid_size);
  this->get_parameter("plane_distance_threshold", plane_distance_threshold);
  this->get_parameter("min_cluster_size", min_cluster_size);
  this->get_parameter("max_pass_filter_fm", max_pass_filter_fm);
  this->get_parameter("voxel_grid_size_fm", voxel_grid_size_fm);
  this->get_parameter("min_cluster_size_fm", min_cluster_size_fm);

  RCLCPP_INFO(this->get_logger(), "max_pass_filter: %f", max_pass_filter);
  RCLCPP_INFO(this->get_logger(), "voxel_grid_size: %f", voxel_grid_size);
  RCLCPP_INFO(this->get_logger(), "plane_distance_threshold: %f", plane_distance_threshold);
  RCLCPP_INFO(this->get_logger(), "min_cluster_size: %d", min_cluster_size);
  RCLCPP_INFO(this->get_logger(), "max_pass_filter_fm: %f", max_pass_filter_fm);
  RCLCPP_INFO(this->get_logger(), "voxel_grid_size_fm: %f", voxel_grid_size_fm);
  RCLCPP_INFO(this->get_logger(), "min_cluster_size_fm: %d", min_cluster_size_fm);

  pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  rclcpp::QoS qos_profile(5);                                      // History depth of 5, as specified
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT); // Best effort reliability
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);      // Volatile durability

  rclcpp::CallbackGroup::SharedPtr cb_group1 = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = cb_group1;

  rclcpp::CallbackGroup::SharedPtr cb_group2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  get_all_box_detail_srv = this->create_service<bin_packing_msgs::srv::GetAllBoxDetail>(
      "get_all_box_detail",
      std::bind(&BoxFinder::getAllBoxDetailServiceCB, this, std::placeholders::_1, std::placeholders::_2));
  get_fm_box_detail_srv = this->create_service<bin_packing_msgs::srv::GetFMBoxDetail>(
      "get_fm_box_detail",
      std::bind(&BoxFinder::getFMBoxDetailServiceCB, this, std::placeholders::_1, std::placeholders::_2));
  // pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //     "/camera/camera/depth/color/points", qos_profile, std::bind(&BoxFinder::pointcloudCB, this, std::placeholders::_1), options);
  processed_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_pointcloud", 5);
  // Transform broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  RCLCPP_INFO(this->get_logger(), "BoxFinder node has been initialized");
}

void BoxFinder::getFMBoxDetailServiceCB(const std::shared_ptr<bin_packing_msgs::srv::GetFMBoxDetail::Request> request,
                                         const std::shared_ptr<bin_packing_msgs::srv::GetFMBoxDetail::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Got request to get box detail at Flipping Mechanism");
  int boxes_detected = -1;
  int successful_detection = 0;
  BoxDetection flipping_mechanism_box;

  for (int i = 0; i < 10; i++)
  {

    auto camera_message = sensor_msgs::msg::PointCloud2();
    rclcpp::wait_for_message(camera_message, this->shared_from_this(), "/camera/camera/depth/color/points", std::chrono::seconds(1));
    pcl::fromROSMsg(camera_message, *pcl_cloud);

    // Apply Z-axis filtering
    pcl::PassThrough<pcl::PointXYZ> pass_filter;
    pass_filter.setInputCloud(pcl_cloud);
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(0.0, max_pass_filter_fm); // Keep points where Z is between 0 and 50 cm
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass_filter.filter(*cloud_pass_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud_pass_filtered);
    voxel_filter.setLeafSize(voxel_grid_size_fm, voxel_grid_size_fm, voxel_grid_size_fm); // Set the voxel grid size (in meters)
    voxel_filter.filter(*cloud_filtered);

    // Clustering to find individual boxes
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02);           // Tolerance for cluster size in meters
    ec.setMinClusterSize(min_cluster_size_fm); // Minimum number of points per cluster
    ec.setMaxClusterSize(25000);            // Maximum number of points per cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    int cluster_id = 0;

    // Iterate over each detected cluster (each box)
    if (cluster_indices.size() == 0)
    {
      RCLCPP_INFO(this->get_logger(), "No boxes detection");
      publishProcessedPointcloud(cloud_filtered);
      response->success = false;
      return;
    }
    if (cluster_indices.size() > 1)
    {
      RCLCPP_INFO(this->get_logger(), "More than 1 box detected, skipping");
      publishProcessedPointcloud(cloud_filtered);
      continue;
    }
    if (cluster_indices.size() != boxes_detected && boxes_detected != -1)
    {
      RCLCPP_INFO(this->get_logger(), "Inconsistent box detection, expected %d boxes instead of %d, skipping", boxes_detected, cluster_indices.size());
      continue;
    }
    if (boxes_detected == -1)
    {
      RCLCPP_INFO(this->get_logger(), "Reserving %d box detection in memory", cluster_indices.size());
      boxes_detected = cluster_indices.size();
    }
    for (const auto &cluster : cluster_indices)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto &idx : cluster.indices)
        cloud_cluster->points.push_back(cloud_filtered->points[idx]);

      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      // Compute OBB for each cluster
      pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
      feature_extractor.setInputCloud(cloud_cluster);
      feature_extractor.compute();

      pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
      Eigen::Matrix3f rotational_matrix_OBB;
      feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

      // Calculate dimensions
      float length = max_point_OBB.x - min_point_OBB.x;
      float width = max_point_OBB.y - min_point_OBB.y;
      float height = max_point_OBB.z - min_point_OBB.z;
      RCLCPP_INFO(this->get_logger(), "detection %d, height: %.3f, max z: %.3f, min z: %.3f, pose z: %.3f", i, height, max_point_OBB.z, min_point_OBB.z, position_OBB.z);

      // Extract Yaw (Rotation around Z-axis) and convert to degrees
      float yaw_radians = std::atan2(rotational_matrix_OBB(1, 0), rotational_matrix_OBB(0, 0));
      float yaw_degrees = yaw_radians * 180.0 / M_PI; // Convert radians to degrees

      if (flipping_mechanism_box.first_yaw)
        flipping_mechanism_box.first_yaw = false;
      else
      {
        double yaw_diff = std::abs(yaw_degrees - flipping_mechanism_box.last_yaw);
        if (yaw_diff >= 170.0 && yaw_diff <= 190.0)
        {
          // RCLCPP_INFO(this->get_logger(), "Yaw correction is needed, current yaw: %f, last yaw: %f", yaw_degrees, flipping_mechanism_box.last_yaw);
          if (yaw_degrees > flipping_mechanism_box.last_yaw)
          {
            yaw_degrees -= 180.0;
          }
          else
          {
            yaw_degrees += 180.0;
          }
        }
      }
      // RCLCPP_INFO(this->get_logger(), "detection %d, box: %d, Yaw: %f", i, cluster_id, yaw_degrees);

      flipping_mechanism_box.last_yaw = yaw_degrees;
      flipping_mechanism_box.length_sum += length;
      flipping_mechanism_box.width_sum += width;
      flipping_mechanism_box.height_sum += height;
      flipping_mechanism_box.yaw_sum += yaw_degrees;
      flipping_mechanism_box.center_x_sum += position_OBB.x;
      flipping_mechanism_box.center_y_sum += position_OBB.y;
      flipping_mechanism_box.center_z_sum += position_OBB.z;

      cluster_id++;
    }

    successful_detection++;

    // Convert the downsampled PCL point cloud back to ROS2 PointCloud2 message
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_filtered, output_msg);
    output_msg.header.frame_id = "camera_depth_optical_frame";
    output_msg.header.stamp = this->get_clock()->now();
    processed_pointcloud_pub->publish(output_msg);
  }
  if (successful_detection <= 5)
  {
    RCLCPP_INFO(this->get_logger(), "Less then 5 successful detection, returning false");
    response->success = false;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "successful detection: %d", successful_detection);
  flipping_mechanism_box.length = flipping_mechanism_box.length_sum / successful_detection;
  flipping_mechanism_box.width = flipping_mechanism_box.width_sum / successful_detection;
  flipping_mechanism_box.height = flipping_mechanism_box.height_sum / successful_detection;
  flipping_mechanism_box.yaw = flipping_mechanism_box.yaw_sum / successful_detection;
  flipping_mechanism_box.center_x = flipping_mechanism_box.center_x_sum / successful_detection;
  flipping_mechanism_box.center_y = flipping_mechanism_box.center_y_sum / successful_detection;
  flipping_mechanism_box.center_z = flipping_mechanism_box.center_z_sum / successful_detection;

  double length = flipping_mechanism_box.length;
  double width = flipping_mechanism_box.width;
  double height = flipping_mechanism_box.height;
  double yaw = flipping_mechanism_box.yaw;
  double center_x = flipping_mechanism_box.center_x;
  double center_y = flipping_mechanism_box.center_y;
  double center_z = flipping_mechanism_box.center_z;
  // Output dimensions, yaw in degrees, and distance to object center
  RCLCPP_INFO(this->get_logger(), "Length: %f, Width: %f, Height: %f", length, width, height);
  RCLCPP_INFO(this->get_logger(), "Yaw: %f degrees", yaw);
  RCLCPP_INFO(this->get_logger(), "Distance x: %.3f , y: %.3f, z: %.3f", center_x, center_y, center_z - height);

  // Publish the transform for each detected box
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->get_clock()->now();
  transform_stamped.header.frame_id = "camera_depth_optical_frame";
  transform_stamped.child_frame_id = "flipping_mechanism_box";

  // Set translation (position of the OBB)
  transform_stamped.transform.translation.x = center_x;
  transform_stamped.transform.translation.y = center_y;
  transform_stamped.transform.translation.z = center_z - 0.5 * height;

  // Convert yaw (rotation around Z) into quaternion
  tf2::Quaternion quat;
  quat.setRPY(0, 0, yaw * M_PI / 180.0); // Only yaw is needed (rotation around Z-axis)
  transform_stamped.transform.rotation.x = quat.x();
  transform_stamped.transform.rotation.y = quat.y();
  transform_stamped.transform.rotation.z = quat.z();
  transform_stamped.transform.rotation.w = quat.w();

  // Broadcast the transform
  tf_broadcaster_->sendTransform(transform_stamped);

  response->box.shape.length = length;
  response->box.shape.width = width;
  response->box.shape.height = height;
  response->box.pose.position.x = center_x;
  response->box.pose.position.y = center_y;
  response->box.pose.position.z = center_z - 0.5 * height;
  response->box.pose.orientation.x = quat.x();
  response->box.pose.orientation.y = quat.y();
  response->box.pose.orientation.z = quat.z();
  response->box.pose.orientation.w = quat.w();
  response->box.yaw = yaw;

  response->success = true;
  return;
}

void BoxFinder::getAllBoxDetailServiceCB(const std::shared_ptr<bin_packing_msgs::srv::GetAllBoxDetail::Request> request,
                                         const std::shared_ptr<bin_packing_msgs::srv::GetAllBoxDetail::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Got request to get all box detail");
  int boxes_detected = -1;
  int successful_detection = 0;
  std::vector<BoxDetection> all_detected_box;

  for (int i = 0; i < 10; i++)
  {

    auto camera_message = sensor_msgs::msg::PointCloud2();
    rclcpp::wait_for_message(camera_message, this->shared_from_this(), "/camera/camera/depth/color/points", std::chrono::seconds(1));
    pcl::fromROSMsg(camera_message, *pcl_cloud);

    // Apply Z-axis filtering
    pcl::PassThrough<pcl::PointXYZ> pass_filter;
    pass_filter.setInputCloud(pcl_cloud);
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(0.0, max_pass_filter); // Keep points where Z is between 0 and 70 cm
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass_filter.filter(*cloud_pass_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud_pass_filtered);
    voxel_filter.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size); // Set the voxel grid size (in meters)
    voxel_filter.filter(*cloud_filtered);

    // Plane segmentation to remove the table top
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(plane_distance_threshold); // Set threshold for plane detection
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    // Extract the inliers (the plane)
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true); // True to remove the plane

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_plane(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*cloud_without_plane);

    // Clustering to find individual boxes
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_without_plane);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02);           // Tolerance for cluster size in meters
    ec.setMinClusterSize(min_cluster_size); // Minimum number of points per cluster
    ec.setMaxClusterSize(25000);            // Maximum number of points per cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_without_plane);
    ec.extract(cluster_indices);

    int cluster_id = 0;

    // Iterate over each detected cluster (each box)
    if (cluster_indices.size() == 0)
    {
      RCLCPP_INFO(this->get_logger(), "No boxes detection");
      publishProcessedPointcloud(cloud_without_plane);
      response->success = false;
      return;
    }
    if (cluster_indices.size() != boxes_detected && boxes_detected != -1)
    {
      RCLCPP_INFO(this->get_logger(), "Inconsistent box detection, expected %d boxes instead of %d, skipping", boxes_detected, cluster_indices.size());
      continue;
    }
    if (boxes_detected == -1)
    {
      RCLCPP_INFO(this->get_logger(), "Reserving %d box detection in memory", cluster_indices.size());
      all_detected_box.resize(cluster_indices.size());
      boxes_detected = cluster_indices.size();
    }
    for (const auto &cluster : cluster_indices)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto &idx : cluster.indices)
        cloud_cluster->points.push_back(cloud_without_plane->points[idx]);

      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      // Compute OBB for each cluster
      pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
      feature_extractor.setInputCloud(cloud_cluster);
      feature_extractor.compute();

      pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
      Eigen::Matrix3f rotational_matrix_OBB;
      feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

      // Calculate dimensions
      float length = max_point_OBB.x - min_point_OBB.x;
      float width = max_point_OBB.y - min_point_OBB.y;
      float height = max_point_OBB.z - min_point_OBB.z;
      RCLCPP_INFO(this->get_logger(), "detection %d, height: %.3f, max z: %.3f, min z: %.3f, pose z: %.3f", i, height, max_point_OBB.z, min_point_OBB.z, position_OBB.z);

      // Extract Yaw (Rotation around Z-axis) and convert to degrees
      float yaw_radians = std::atan2(rotational_matrix_OBB(1, 0), rotational_matrix_OBB(0, 0));
      float yaw_degrees = yaw_radians * 180.0 / M_PI; // Convert radians to degrees

      if (all_detected_box[cluster_id].first_yaw)
        all_detected_box[cluster_id].first_yaw = false;
      else
      {
        double yaw_diff = std::abs(yaw_degrees - all_detected_box[cluster_id].last_yaw);
        if (yaw_diff >= 170.0 && yaw_diff <= 190.0)
        {
          // RCLCPP_INFO(this->get_logger(), "Yaw correction is needed, current yaw: %f, last yaw: %f", yaw_degrees, all_detected_box[cluster_id].last_yaw);
          if (yaw_degrees > all_detected_box[cluster_id].last_yaw)
          {
            yaw_degrees -= 180.0;
          }
          else
          {
            yaw_degrees += 180.0;
          }
        }
      }
      // RCLCPP_INFO(this->get_logger(), "detection %d, box: %d, Yaw: %f", i, cluster_id, yaw_degrees);

      all_detected_box[cluster_id].last_yaw = yaw_degrees;
      all_detected_box[cluster_id].length_sum += length;
      all_detected_box[cluster_id].width_sum += width;
      all_detected_box[cluster_id].height_sum += height;
      all_detected_box[cluster_id].yaw_sum += yaw_degrees;
      all_detected_box[cluster_id].center_x_sum += position_OBB.x;
      all_detected_box[cluster_id].center_y_sum += position_OBB.y;
      all_detected_box[cluster_id].center_z_sum += position_OBB.z;
      // all_detected_box[cluster_id].yaw_detection.push_back(yaw_degrees);

      cluster_id++;
    }

    successful_detection++;

    // Convert the downsampled PCL point cloud back to ROS2 PointCloud2 message
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_without_plane, output_msg);
    output_msg.header.frame_id = "camera_depth_optical_frame";
    output_msg.header.stamp = this->get_clock()->now();
    processed_pointcloud_pub->publish(output_msg);
  }
  if (successful_detection <= 5)
  {
    RCLCPP_INFO(this->get_logger(), "Less then 5 successful detection, returning false");
    response->success = false;
    return;
  }
  for (int i = 0; i < boxes_detected; i++)
  {
    RCLCPP_INFO(this->get_logger(), "successful detection: %d", successful_detection);
    all_detected_box[i].length = all_detected_box[i].length_sum / successful_detection;
    all_detected_box[i].width = all_detected_box[i].width_sum / successful_detection;
    all_detected_box[i].height = all_detected_box[i].height_sum / successful_detection;
    all_detected_box[i].yaw = all_detected_box[i].yaw_sum / successful_detection;
    all_detected_box[i].center_x = all_detected_box[i].center_x_sum / successful_detection;
    all_detected_box[i].center_y = all_detected_box[i].center_y_sum / successful_detection;
    all_detected_box[i].center_z = all_detected_box[i].center_z_sum / successful_detection;

    double length = all_detected_box[i].length;
    double width = all_detected_box[i].width;
    double height = all_detected_box[i].height;
    double yaw = all_detected_box[i].yaw;
    double center_x = all_detected_box[i].center_x;
    double center_y = all_detected_box[i].center_y;
    double center_z = all_detected_box[i].center_z;
    // Output dimensions, yaw in degrees, and distance to object center
    RCLCPP_INFO(this->get_logger(), "Box %d - Length: %f, Width: %f, Height: %f", i, length, width, height);
    RCLCPP_INFO(this->get_logger(), "Box %d - Yaw: %f degrees", i, yaw);
    RCLCPP_INFO(this->get_logger(), "Box %d - Distance x: %.3f , y: %.3f, z: %.3f", i, center_x, center_y, center_z - height);

    // Publish the transform for each detected box
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "camera_depth_optical_frame";
    transform_stamped.child_frame_id = "box_" + std::to_string(i);

    // Set translation (position of the OBB)
    transform_stamped.transform.translation.x = center_x;
    transform_stamped.transform.translation.y = center_y;
    transform_stamped.transform.translation.z = center_z - 0.5 * height;

    // Convert yaw (rotation around Z) into quaternion
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw * M_PI / 180.0); // Only yaw is needed (rotation around Z-axis)
    transform_stamped.transform.rotation.x = quat.x();
    transform_stamped.transform.rotation.y = quat.y();
    transform_stamped.transform.rotation.z = quat.z();
    transform_stamped.transform.rotation.w = quat.w();

    // Broadcast the transform
    tf_broadcaster_->sendTransform(transform_stamped);

    bin_packing_msgs::msg::BoxDetail temp;
    temp.shape.length = length;
    temp.shape.width = width;
    temp.shape.height = height;
    temp.shape.name = "item_" + std::to_string(i);
    temp.pose.position.x = center_x;
    temp.pose.position.y = center_y;
    temp.pose.position.z = center_z - 0.5 * height;
    temp.pose.orientation.x = quat.x();
    temp.pose.orientation.y = quat.y();
    temp.pose.orientation.z = quat.z();
    temp.pose.orientation.w = quat.w();
    response->boxes.push_back(temp);
  }

  response->success = true;
  return;
}

void BoxFinder::publishProcessedPointcloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> data)
{
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*data, output_msg);
  output_msg.header.frame_id = "camera_depth_optical_frame";
  output_msg.header.stamp = this->get_clock()->now();
  processed_pointcloud_pub->publish(output_msg);
}

void BoxFinder::pointcloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

  // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *pcl_cloud);
  /*
  // Apply VoxelGrid filter for downsampling
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(pcl_cloud);
  voxel_filter.setLeafSize(0.005f, 0.005f, 0.005f); // Set the voxel grid size (in meters)

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_filter.filter(*cloud_filtered);

  // Plane segmentation to remove the table top
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01); // Set threshold for plane detection
  seg.setInputCloud(cloud_filtered);
  seg.segment(*inliers, *coefficients);

  // Extract the inliers (the plane)
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true); // True to remove the plane

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_plane(new pcl::PointCloud<pcl::PointXYZ>);
  extract.filter(*cloud_without_plane);

  // Clustering to find individual boxes
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_without_plane);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.02); // Tolerance for cluster size in meters
  ec.setMinClusterSize(300);    // Minimum number of points per cluster
  ec.setMaxClusterSize(25000);  // Maximum number of points per cluster
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_without_plane);
  ec.extract(cluster_indices);

  int cluster_id = 0;

  // Iterate over each detected cluster (each box)
  for (const auto &cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &idx : cluster.indices)
      cloud_cluster->points.push_back(cloud_without_plane->points[idx]);

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // Compute OBB for each cluster
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_cluster);
    feature_extractor.compute();

    pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    // Calculate dimensions
    float length = max_point_OBB.x - min_point_OBB.x;
    float width = max_point_OBB.y - min_point_OBB.y;
    float height = max_point_OBB.z - min_point_OBB.z;

    // Extract Yaw (Rotation around Z-axis) and convert to degrees
    float yaw_radians = std::atan2(rotational_matrix_OBB(1, 0), rotational_matrix_OBB(0, 0));
    float yaw_degrees = yaw_radians * 180.0 / M_PI; // Convert radians to degrees

    // Normalize yaw to the range [0, 360) degrees
    if (yaw_degrees < 0)
    {
      yaw_degrees += 360.0;
    }

    // Output dimensions, yaw in degrees, and distance to object center
    RCLCPP_INFO(this->get_logger(), "Box %d - Length: %f, Width: %f, Height: %f", cluster_id, length, width, height);
    RCLCPP_INFO(this->get_logger(), "Box %d - Yaw: %f degrees", cluster_id, yaw_degrees);
    RCLCPP_INFO(this->get_logger(), "Box %d - Distance x: %.3f , y: %.3f, z: %.3f", cluster_id, position_OBB.x, position_OBB.y, position_OBB.z + height);

    // Publish the transform for each detected box
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "camera_depth_optical_frame";
    transform_stamped.child_frame_id = "box_" + std::to_string(cluster_id);

    // Set translation (position of the OBB)
    transform_stamped.transform.translation.x = position_OBB.x;
    transform_stamped.transform.translation.y = position_OBB.y;
    transform_stamped.transform.translation.z = position_OBB.z + height;

    // Convert yaw (rotation around Z) into quaternion
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw_degrees * M_PI / 180.0); // Only yaw is needed (rotation around Z-axis)
    transform_stamped.transform.rotation.x = quat.x();
    transform_stamped.transform.rotation.y = quat.y();
    transform_stamped.transform.rotation.z = quat.z();
    transform_stamped.transform.rotation.w = quat.w();

    // Broadcast the transform
    tf_broadcaster_->sendTransform(transform_stamped);
    cluster_id++;
  }

  // Convert the downsampled PCL point cloud back to ROS2 PointCloud2 message
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*cloud_without_plane, output_msg);
  output_msg.header = msg->header;
  processed_pointcloud_pub->publish(output_msg);
  */
}
