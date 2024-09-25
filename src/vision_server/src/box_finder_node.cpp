#include "vision_server/box_finder.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<BoxFinder>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
