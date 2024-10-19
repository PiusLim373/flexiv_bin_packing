#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "bin_packing_msgs/srv/get_all_box_detail.hpp"
#include "bin_packing_msgs/srv/get_fm_box_detail.hpp"
#include "bin_packing_msgs/msg/box_detail.hpp"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/kdtree/kdtree.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cstdio>
#include <vector>

class BoxFinder : public rclcpp::Node
{
public:
    BoxFinder();

    struct BoxDetection
    {
        // Members to store dimensions of the box
        bool first_yaw = true;
        double last_yaw = -1.0;
        double width_sum = 0.0;
        double length_sum = 0.0;
        double height_sum = 0.0;
        double yaw_sum = 0.0;
        double center_x_sum = 0.0;
        double center_y_sum = 0.0;
        double center_z_sum = 0.0;

        double width;
        double length;
        double height;
        double yaw;

        double center_x;
        double center_y;
        double center_z;

        // Member function to print the dimensions and calculations
        void printDetails() const
        {
            std::cout << "Box dimensions: " << std::endl;
            std::cout << "Width: " << width << std::endl;
            std::cout << "length: " << length << std::endl;
            std::cout << "height: " << height << std::endl;
        }
    };

private:
    double max_pass_filter_global, voxel_grid_size_global, plane_distance_threshold_global;
    int min_cluster_size_global;
    int averaging_tries, min_averaging_tries;

    double max_pass_filter_fm, voxel_grid_size_fm;
    int min_cluster_size_fm;
    pcl::PointCloud<pcl::PointXYZ> camera_pcl;
    rclcpp::Service<bin_packing_msgs::srv::GetAllBoxDetail>::SharedPtr get_all_box_detail_srv;
    rclcpp::Service<bin_packing_msgs::srv::GetFMBoxDetail>::SharedPtr get_fm_box_detail_srv;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_pointcloud_pub;
    void getAllBoxDetailServiceCB(const std::shared_ptr<bin_packing_msgs::srv::GetAllBoxDetail::Request> request,
                                  const std::shared_ptr<bin_packing_msgs::srv::GetAllBoxDetail::Response> response);
    void getFMBoxDetailServiceCB(const std::shared_ptr<bin_packing_msgs::srv::GetFMBoxDetail::Request> request,
                                 const std::shared_ptr<bin_packing_msgs::srv::GetFMBoxDetail::Response> response);
    void pointcloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud;
    void publishProcessedPointcloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> data);
    std::vector<pcl::PointIndices> get_clusters(double max_pass_filter, double voxel_grid_size, double plane_distance_threshold, int min_cluster_size, bool enable_plane_segmentation, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_without_plane);
};
