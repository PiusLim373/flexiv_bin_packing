#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "bin_packing_msgs/srv/get_pixel_point.hpp"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <cstdio>

class PCLUtils : public rclcpp::Node
{
public:
    PCLUtils();

private:
    pcl::PointCloud<pcl::PointXYZ> camera_pcl;
    rclcpp::Service<bin_packing_msgs::srv::GetPixelPoint>::SharedPtr get_pixel_point_srv;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
    void getPixelPointServiceCB(const std::shared_ptr<bin_packing_msgs::srv::GetPixelPoint::Request> request,
                                const std::shared_ptr<bin_packing_msgs::srv::GetPixelPoint::Response> response);
    void pointcloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
};
