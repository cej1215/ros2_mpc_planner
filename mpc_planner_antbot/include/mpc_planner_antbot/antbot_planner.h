#ifndef ANTBOT_PLANNER_H
#define ANTBOT_PLANNER_H

#include <mpc_planner/planner.h>
#include <mpc_planner_solver/state.h>
#include <mpc_planner_types/realtime_data.h>
#include <ros_tools/profiling.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/empty.hpp>
#include <robot_localization/srv/set_pose.hpp>
#include <boost/thread/mutex.hpp>
#include <memory>
#include <string>
#include <vector>

using namespace MPCPlanner;

class AntbotReconfigure;

class AntbotPlanner : public rclcpp::Node
{
public:
    AntbotPlanner();
    ~AntbotPlanner() override;

    void initialize();
    void initializeSubscribersAndPublishers(rclcpp::Node &nh);
    void startEnvironment();
    bool setPlan(const std::vector<geometry_msgs::msg::PoseStamped> &orig_global_plan);
    bool computeVelocityCommands(geometry_msgs::msg::Twist &cmd_vel);
    bool isGoalReached();
    void Loop(geometry_msgs::msg::Twist &cmd_vel);

    void stateCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void costmapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void collisionCallback(const std_msgs::msg::Float64::ConstSharedPtr &msg);
    void reset(bool success = true);
    void publishPose();

private:
    bool checkProgress();
    void resetProgressBaseline();

    bool _progress_baseline_initialized{false};
    double _progress_baseline_x{0.0};
    double _progress_baseline_y{0.0};
    rclcpp::Time _progress_baseline_time;
    double _required_movement_radius{0.1};
    double _movement_time_allowance{60.0};

    nav2_costmap_2d::Costmap2D costmap_;
    bool _costmap_received{false};
    bool initialized_{false};
    bool done_{false};
    bool _enable_output{false};
    bool _state_received{false};
    std::vector<geometry_msgs::msg::PoseStamped> global_plan_;

    std::unique_ptr<Planner> _planner;
    RealTimeData _data;
    State _state;
    rclcpp::TimerBase::SharedPtr _timer;
    std::unique_ptr<AntbotReconfigure> _reconfigure;
    RosTools::Timer _timeout_timer;
    boost::mutex _reset_mutex;
    rclcpp::CallbackGroup::SharedPtr _service_callback_group;
    rclcpp::executors::SingleThreadedExecutor _service_executor;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _state_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _costmap_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _collisions_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _reset_simulation_pub;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _reset_simulation_client;
    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr _reset_ekf_client;
    std_srvs::srv::Empty::Request _reset_msg;
    robot_localization::srv::SetPose::Request _reset_pose_msg;

    bool isPathTheSame(const nav_msgs::msg::Path::ConstSharedPtr &path);
    void visualize();
};

#endif
