#include "vision_server/pcl_utils.hpp"

PCLUtils::PCLUtils() : Node("pcl_utils")
{
  rclcpp::QoS qos_profile(5);                                      // History depth of 5, as specified
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT); // Best effort reliability
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);      // Volatile durability
  rclcpp::CallbackGroup::SharedPtr cb_group1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = cb_group1;
  get_pixel_point_srv = this->create_service<bin_packing_msgs::srv::GetPixelPoint>(
      "get_pixel_point",
      std::bind(&PCLUtils::getPixelPointServiceCB, this, std::placeholders::_1, std::placeholders::_2));
  pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/color/points", qos_profile, std::bind(&PCLUtils::pointcloudCB, this, std::placeholders::_1), options);
}

void PCLUtils::getPixelPointServiceCB(const std::shared_ptr<bin_packing_msgs::srv::GetPixelPoint::Request> request,
                                      const std::shared_ptr<bin_packing_msgs::srv::GetPixelPoint::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Got reques to get point at (%d, %d)", request->u, request->v);
  pcl::PointXYZ point;
  try
  {
    point = camera_pcl.at(request->u, request->v);
  }
  catch (pcl::IsNotDenseException &err)
  {
    RCLCPP_ERROR(this->get_logger(), "cloud not valid, have you get depth data before?");
    response->success = false;
    return;
  }
  RCLCPP_INFO(this->get_logger(), "x: [%f]; y: [%f]; z: [%f]", point.x, point.y, point.z);
  if ( std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
  {
    RCLCPP_ERROR(this->get_logger(), "couldn't determine depth, probably too far");
    response->success = false;
    return;
  }
  response->point.x = point.x;
  response->point.y = point.y;
  response->point.z = point.z;
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "Successfullt getting pixel pose, returning");
  return;
}

void PCLUtils::pointcloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::fromROSMsg(*msg, camera_pcl);
}
