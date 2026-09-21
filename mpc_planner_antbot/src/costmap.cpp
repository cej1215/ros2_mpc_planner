// src/local_costmap_node.cpp
#include <memory>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto local_costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(local_costmap->get_node_base_interface());
  

  executor.spin();

  rclcpp::shutdown();
  return 0;
}

//
// #include <memory>
// #include "nav2_costmap_2d/costmap_2d_ros.hpp"
// #include "rclcpp/rclcpp.hpp"

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);

//   rclcpp::NodeOptions options;

//   auto local_costmap =
//       std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(local_costmap->get_node_base_interface());

//   executor.spin();

//   rclcpp::shutdown();
//   return 0;
// }