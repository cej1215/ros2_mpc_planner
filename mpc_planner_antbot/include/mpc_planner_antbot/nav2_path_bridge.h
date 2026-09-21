#pragma once

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace mpc_planner_antbot
{

class Nav2PathBridge : public rclcpp::Node
{
public:
    explicit Nav2PathBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    using ComputePath = nav2_msgs::action::ComputePathToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;

    void goalCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr goal);
    void pathCallback(nav_msgs::msg::Path::ConstSharedPtr path);

    std::string planner_id_;
    std::string target_frame_;
    bool request_pending_{false};
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp_action::Client<ComputePath>::SharedPtr path_client_;
};

}  // namespace mpc_planner_antbot
