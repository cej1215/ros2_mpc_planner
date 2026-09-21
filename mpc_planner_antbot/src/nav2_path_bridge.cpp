#include <mpc_planner_antbot/nav2_path_bridge.h>

#include <functional>
#include <memory>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mpc_planner_antbot
{

Nav2PathBridge::Nav2PathBridge(const rclcpp::NodeOptions &options)
    : Node("nav2_path_bridge", options),
      tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_)
{
    planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");
    const auto action_name = declare_parameter<std::string>(
        "planner_action_name", "/compute_path_to_pose");
    path_client_ = rclcpp_action::create_client<ComputePath>(this, action_name);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/output/goal", 1);
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/input/goal", 1,
        std::bind(&Nav2PathBridge::goalCallback, this, std::placeholders::_1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/output/reference_path", 1);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "~/input/path", 1, std::bind(&Nav2PathBridge::pathCallback, this, std::placeholders::_1));
}

void Nav2PathBridge::goalCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr goal)
{
    if (goal->header.frame_id.empty())
    {
        RCLCPP_WARN(get_logger(), "Goal frame_id is empty; skipping request.");
        return;
    }
    if (request_pending_)
    {
        RCLCPP_WARN(get_logger(), "Path request is still pending; resend the goal after it finishes.");
        return;
    }
    if (!path_client_->action_server_is_ready())
    {
        RCLCPP_WARN(get_logger(), "ComputePathToPose server is unavailable; resend the goal once Nav2 is active.");
        return;
    }

    geometry_msgs::msg::PoseStamped transformed_goal = *goal;
    try
    {
        if (goal->header.frame_id != target_frame_)
        {
            const auto transform = tf_buffer_.lookupTransform(
                target_frame_, goal->header.frame_id, tf2::TimePointZero);
            tf2::doTransform(*goal, transformed_goal, transform);
        }
    }
    catch (const tf2::TransformException &exception)
    {
        RCLCPP_WARN(get_logger(), "Cannot transform goal to %s: %s",
                    target_frame_.c_str(), exception.what());
        return;
    }

    ComputePath::Goal request;
    request.goal = *goal;
    request.planner_id = planner_id_;
    request.use_start = false;

    auto options = rclcpp_action::Client<ComputePath>::SendGoalOptions();
    options.goal_response_callback = [this](GoalHandle::SharedPtr handle)
    {
        if (!handle)
        {
            request_pending_ = false;
            RCLCPP_ERROR(get_logger(), "Nav2 rejected the path request.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Nav2 accepted the path request.");
    };
    options.result_callback = [this](const GoalHandle::WrappedResult &result)
    {
        request_pending_ = false;
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_ERROR(get_logger(), "Nav2 path request failed or was canceled (result code %d).",
                         static_cast<int>(result.code));
            return;
        }
        if (result.result->path.poses.empty())
        {
            RCLCPP_WARN(get_logger(), "Nav2 returned an empty path.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Nav2 path request succeeded with %zu poses.",
                    result.result->path.poses.size());
    };

    request_pending_ = true;
    try
    {
        goal_pub_->publish(transformed_goal);
        path_client_->async_send_goal(request, options);
        RCLCPP_INFO(get_logger(), "Goal received; requesting a path from Nav2.");
    }
    catch (const std::exception &exception)
    {
        request_pending_ = false;
        RCLCPP_ERROR(get_logger(), "Cannot send path request: %s", exception.what());
    }
}

void Nav2PathBridge::pathCallback(nav_msgs::msg::Path::ConstSharedPtr path)
{
    if (path->header.frame_id.empty())
    {
        RCLCPP_WARN(get_logger(), "Path frame_id is empty; skipping transform.");
        return;
    }

    // Nav2 paths use a single coordinate frame for all poses.
    for (const auto &pose : path->poses)
    {
        if (!pose.header.frame_id.empty() && pose.header.frame_id != path->header.frame_id)
        {
            RCLCPP_WARN(get_logger(), "Path contains inconsistent pose frames; skipping transform.");
            return;
        }
    }

    nav_msgs::msg::Path output;
    output.header.frame_id = target_frame_;
    output.header.stamp = now();
    output.poses.reserve(path->poses.size());

    try
    {
        if (path->header.frame_id == target_frame_)
        {
            output.poses = path->poses;
        }
        else if (!path->poses.empty())
        {
            // Reuse one latest transform so the entire path remains consistent.
            const auto transform = tf_buffer_.lookupTransform(
                target_frame_, path->header.frame_id, tf2::TimePointZero);
            output.header.stamp = transform.header.stamp;
            for (const auto &pose : path->poses)
            {
                geometry_msgs::msg::PoseStamped transformed_pose;
                tf2::doTransform(pose, transformed_pose, transform);
                output.poses.push_back(transformed_pose);
            }
        }

        for (auto &pose : output.poses)
            pose.header = output.header;

        path_pub_->publish(output);
    }
    catch (const tf2::TransformException &exception)
    {
        RCLCPP_WARN(get_logger(), "Cannot transform path to %s: %s",
                    target_frame_.c_str(), exception.what());
    }
}

}  // namespace mpc_planner_antbot

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mpc_planner_antbot::Nav2PathBridge>());
    rclcpp::shutdown();
    return 0;
}
