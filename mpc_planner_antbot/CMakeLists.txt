cmake_minimum_required(VERSION 3.8)
project(mpc_planner_antbot)

add_compile_options(-std=c++17)

set(DEPENDENCIES
  rclcpp
  mpc_planner
  mpc_planner_solver
  mpc_planner_util
  mpc_planner_types
  mpc_planner_msgs
  ros_tools
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  nav2_costmap_2d
  nav2_util
  lifecycle_msgs
  robot_localization
)

find_package(ament_cmake REQUIRED)
foreach(pkg IN LISTS DEPENDENCIES)
  find_package(${pkg} REQUIRED)
endforeach()
find_package(Boost REQUIRED COMPONENTS thread)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(antbot_planner src/antbot_planner.cpp)
target_include_directories(antbot_planner PRIVATE include)
ament_target_dependencies(antbot_planner ${DEPENDENCIES})
target_link_libraries(antbot_planner Boost::thread yaml-cpp)

add_executable(costmap_node src/costmap.cpp)
ament_target_dependencies(costmap_node rclcpp nav2_costmap_2d)

add_executable(nav2_path_bridge src/nav2_path_bridge.cpp)
target_include_directories(nav2_path_bridge PRIVATE include)
ament_target_dependencies(nav2_path_bridge
  rclcpp rclcpp_action nav2_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)

install(TARGETS antbot_planner costmap_node nav2_path_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES} Boost yaml-cpp rclcpp_action nav2_msgs tf2 tf2_geometry_msgs)
ament_package()
