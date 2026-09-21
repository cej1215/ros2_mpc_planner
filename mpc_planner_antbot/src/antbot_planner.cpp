#include <mpc_planner_antbot/antbot_planner.h>
#include <mpc_planner_antbot/antbot_ros2_reconfigure.h>

#include <chrono>
#include <functional>
#include <cmath>
#include <rclcpp/create_timer.hpp>
#include <ros_tools/ros2_wrappers.h>

#include <mpc_planner/planner.h>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/load_yaml.hpp>

#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>
#include <ros_tools/convertions.h>
#include <ros_tools/math.h>
#include <ros_tools/data_saver.h>
#include <ros_tools/spline.h>

#include <std_msgs/msg/empty.hpp>
#include <ros_tools/profiling.h>

using namespace MPCPlanner;

    AntbotPlanner::AntbotPlanner()
        : rclcpp::Node("antbot_planner"), initialized_(false)
    {
        STATIC_NODE_POINTER.init(this);
        initialize();
    }

    void AntbotPlanner::initialize()
    {
        if (!initialized_)
        {
            rclcpp::Node &nh = *this;
            _data.costmap = &costmap_;

            initialized_ = true;

            LOG_INFO("Started ROSNavigation Planner");

            VISUALS.init(this);

            // Initialize the configuration
            Configuration::getInstance().initialize(SYSTEM_CONFIG_PATH(__FILE__, "settings"));

            _required_movement_radius = CONFIG["required_movement_radius"].as<double>(0.1);
            _movement_time_allowance = CONFIG["movement_time_allowance"].as<double>(60.0);

            _data.robot_area = {Disc(0., CONFIG["robot_radius"].as<double>())};

            // Initialize the planner
            _planner = std::make_unique<Planner>();

            // Initialize the ROS interface
            initializeSubscribersAndPublishers(nh);

            startEnvironment();

            _reconfigure = std::make_unique<AntbotReconfigure>(this);

            RosTools::Instrumentor::Get().BeginSession("mpc_planner_antbot");

            LOG_DIVIDER();

            _timer = rclcpp::create_timer(
                this, get_clock(),
                rclcpp::Duration::from_seconds(1.0 / CONFIG["control_frequency"].as<double>()),
                [this]()
                {
                    if (!_state_received || _data.reference_path.x.empty() ||
                        !_costmap_received)
                        return;
                    if (isGoalReached())
                    {
                        _cmd_pub->publish(geometry_msgs::msg::Twist());
                        return;
                    }
                    geometry_msgs::msg::Twist cmd_vel;
                    computeVelocityCommands(cmd_vel);
                });
        }
    }

    AntbotPlanner::~AntbotPlanner()
    {
        LOG_INFO("Stopped Antbot Planner");
        BENCHMARKERS.print();

        RosTools::Instrumentor::Get().EndSession();
    }

    bool AntbotPlanner::setPlan(const std::vector<geometry_msgs::msg::PoseStamped> &orig_global_plan)
    {
        // check if plugin is initialized
        if (!initialized_)
        {
            RCLCPP_ERROR(get_logger(), "planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        // store the global plan
        global_plan_.clear();
        global_plan_ = orig_global_plan;

        // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
        // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.

        // reset goal_reached_ flag
        // goal_reached_ = false;

        return true;
    }

    bool AntbotPlanner::computeVelocityCommands(geometry_msgs::msg::Twist &cmd_vel)
    {
        if (!initialized_)
        {
            RCLCPP_ERROR(get_logger(), "This planner has not been initialized");
            return false;
        }

        auto path = std::make_shared<nav_msgs::msg::Path>();
        path->poses = global_plan_;
        pathCallback(path);

        Loop(cmd_vel);

        return true;
    }

    void AntbotPlanner::initializeSubscribersAndPublishers(rclcpp::Node &nh)
    {
        LOG_INFO("initializeSubscribersAndPublishers");

        _state_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/input/state", rclcpp::SensorDataQoS(),
        std::bind(&AntbotPlanner::stateCallback, this, std::placeholders::_1));

        _goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "~/input/goal", 1,
            std::bind(&AntbotPlanner::goalCallback, this, std::placeholders::_1));

        _path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "~/input/reference_path", 1,
            std::bind(&AntbotPlanner::pathCallback, this, std::placeholders::_1));

        _costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "~/input/costmap", rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&AntbotPlanner::costmapCallback, this, std::placeholders::_1));

        _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
            "~/output/command", 1);

        _pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "~/output/pose", 1);
        
        _collisions_sub = this->create_subscription<std_msgs::msg::Float64>(
            "~/feedback/collisions", 1,
            std::bind(&AntbotPlanner::collisionCallback, this, std::placeholders::_1));

        // Service responses use a separate callback group so synchronous waits also
        // work when reset() is called from a subscription callback.
        _service_callback_group = nh.create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive, false);
        _service_executor.add_callback_group(_service_callback_group, nh.get_node_base_interface());

        // Environment Reset
        _reset_simulation_pub = this->create_publisher<std_msgs::msg::Empty>("/lmpcc/reset_environment", 1);
        _reset_simulation_client = nh.create_client<std_srvs::srv::Empty>("/gazebo/reset_world",
            rmw_qos_profile_services_default, _service_callback_group);
        _reset_ekf_client = nh.create_client<robot_localization::srv::SetPose>("/set_pose",
            rmw_qos_profile_services_default, _service_callback_group);

    }

    void AntbotPlanner::startEnvironment()
    {

        _enable_output = CONFIG["enable_output"].as<bool>();
        LOG_INFO("Environment ready.");
    }

    bool AntbotPlanner::isGoalReached()
    {
        if (!initialized_)
        {
            RCLCPP_ERROR(get_logger(), "This planner has not been initialized");
            return false;
        }

        bool goal_reached = _planner->isObjectiveReached(_state, _data) && !done_; // Activate once
        if (goal_reached)
        {
            LOG_SUCCESS("Goal Reached!");
            done_ = true;
            reset();
        }

        return goal_reached;
    }

    void AntbotPlanner::resetProgressBaseline()
    {
        _progress_baseline_x = _state.get("x");
        _progress_baseline_y = _state.get("y");
        _progress_baseline_time = get_clock()->now();
        _progress_baseline_initialized = true;
    }

    bool AntbotPlanner::checkProgress()
    {
        if (!_progress_baseline_initialized)
        {
            resetProgressBaseline();
            return true;
        }

        // XY displacement only; rotating in place does not refresh the baseline.
        const double distance = std::hypot(
            _state.get("x") - _progress_baseline_x,
            _state.get("y") - _progress_baseline_y);

        if (distance > _required_movement_radius)
        {
            resetProgressBaseline();
            return true;
        }

        // ROS time follows use_sim_time; small movements do not restart the clock.
        const double elapsed_time = (get_clock()->now() - _progress_baseline_time).seconds();
        return !(elapsed_time > _movement_time_allowance);
    }

    void AntbotPlanner::Loop(geometry_msgs::msg::Twist &cmd_vel)
    {

        _data.planning_start_time = std::chrono::system_clock::now();

        LOG_MARK("============= Loop =============");

        if (!checkProgress())
        {
            RCLCPP_WARN(get_logger(),
                "No XY progress greater than %.3f m within %.1f s. Resetting.",
                _required_movement_radius, _movement_time_allowance);
            reset();
            resetProgressBaseline();
        }

        if (CONFIG["debug_output"].as<bool>())
            _state.print();

        auto &loop_benchmarker = BENCHMARKERS.getBenchmarker("loop");
        loop_benchmarker.start();

        auto output = _planner->solveMPC(_state, _data);

        LOG_MARK("Success: " << output.success);

        auto &cmd = cmd_vel;
        if (_enable_output && output.success)
        {
            // Publish the command
            cmd_vel.linear.x = _planner->getSolution(1, "v");  // = x1
            cmd_vel.angular.z = _planner->getSolution(0, "w"); // = u0
            LOG_VALUE_DEBUG("Commanded v", cmd.linear.x);
            LOG_VALUE_DEBUG("Commanded w", cmd.angular.z);
        }
        else
        {
            double deceleration = CONFIG["deceleration_at_infeasible"].as<double>();
            double velocity_after_braking;
            double velocity;
            double dt = 1. / CONFIG["control_frequency"].as<double>();

            velocity = _state.get("v");
            velocity_after_braking = velocity - deceleration * dt;   // Brake with the given deceleration
            cmd_vel.linear.x = std::max(velocity_after_braking, 0.); // Don't drive backwards when braking
            cmd_vel.angular.z = 0.0;
        }
        _cmd_pub->publish(cmd);

        publishPose();

        loop_benchmarker.stop();

        if (CONFIG["recording"]["enable"].as<bool>())
        {

            // Save control inputs
            if (output.success)
            {
                auto &data_saver = _planner->getDataSaver();
                data_saver.AddData("input_a", _state.get("a"));
                data_saver.AddData("input_v", _planner->getSolution(1, "v"));
                data_saver.AddData("input_w", _planner->getSolution(0, "w"));
            }

            _planner->saveData(_state, _data);
        }
        if (output.success)
        {
            _planner->visualize(_state, _data);
            visualize();
        }
        LOG_MARK("============= End Loop =============");
    }

    void AntbotPlanner::stateCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
    {

        _state_received = true;
        _state.set("x", msg->pose.pose.position.x);
        _state.set("y", msg->pose.pose.position.y);
        _state.set("psi", RosTools::quaternionToAngle(msg->pose.pose.orientation));
        _state.set("v", std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) + std::pow(msg->twist.twist.linear.y, 2.)));

        if (std::abs(msg->pose.pose.orientation.x) > (M_PI / 8.) || std::abs(msg->pose.pose.orientation.y) > (M_PI / 8.))
        {
            LOG_WARN("Detected flipped robot. Resetting.");
            reset(false); // Reset without success
        }
    }

    // void ROSNavigationPlanner::statePoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
    // {
    //     LOG_MARK("State callback");

    //     _state.set("x", msg->pose.position.x);
    //     _state.set("y", msg->pose.position.y);
    //     _state.set("psi", msg->pose.orientation.z);
    //     _state.set("v", msg->pose.position.z);

    //     if (std::abs(msg->pose.orientation.x) > (M_PI / 8.) || std::abs(msg->pose.orientation.y) > (M_PI / 8.))
    //     {
    //         LOG_ERROR("Detected flipped robot. Resetting.");
    //         reset(false); // Reset without success
    //     }
    // }

    void AntbotPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
    {
        LOG_MARK("Goal callback");

        _data.goal(0) = msg->pose.position.x;
        _data.goal(1) = msg->pose.position.y;
        _data.goal_received = true;
    }

    bool AntbotPlanner::isPathTheSame(const nav_msgs::msg::Path::ConstSharedPtr &msg)
    {
        // Check if the path is the same
        if (_data.reference_path.x.size() != msg->poses.size())
            return false;

        // Check up to the first two points
        int num_points = std::min(2, (int)_data.reference_path.x.size());
        for (int i = 0; i < num_points; i++)
        {
            if (!_data.reference_path.pointInPath(i, msg->poses[i].pose.position.x, msg->poses[i].pose.position.y))
                return false;
        }
        return true;
    }

    void AntbotPlanner::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
    {
        LOG_MARK("Path callback");
        setPlan(msg->poses);

        int downsample = CONFIG["downsample_path"].as<double>();

        if (isPathTheSame(msg) || msg->poses.size() < downsample + 1)
            return;

        _data.reference_path.clear();

        int count = 0;
        for (auto &pose : msg->poses)
        {
            if (count % downsample == 0 || count == msg->poses.size() - 1) // Todo
            {
                _data.reference_path.x.push_back(pose.pose.position.x);
                _data.reference_path.y.push_back(pose.pose.position.y);
                _data.reference_path.psi.push_back(RosTools::quaternionToAngle(pose.pose.orientation));
            }
            count++;
        }

        // Fit a clothoid on the global path to sample points on the spline from
        // RosTools::Clothoid2D clothoid(_data.reference_path.x, _data.reference_path.y, _data.reference_path.psi, 2.0);
        // _data.reference_path.clear();
        // clothoid.getPointsOnClothoid(_data.reference_path.x, _data.reference_path.y, _data.reference_path.s);

        // Velocity
        /*LOG_VALUE("velocity reference", CONFIG["weights"]["reference_velocity"].as<double>());
        for (size_t i = 0; i < _data.reference_path.x.size(); i++)
        {
            if (i != _data.reference_path.x.size() - 1)
                _data.reference_path.v.push_back(CONFIG["weights"]["reference_velocity"].as<double>());
            else
                _data.reference_path.v.push_back(0.);
        }*/

        _planner->onDataReceived(_data, "reference_path");
    }

    void AntbotPlanner::costmapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        costmap_ = nav2_costmap_2d::Costmap2D(*msg);
        _data.costmap = &costmap_;
        _costmap_received = true;
        _planner->onDataReceived(_data, "costmap");
    }

    void AntbotPlanner::visualize()
    {
        auto &publisher = VISUALS.getPublisher("angle");
        auto &line = publisher.getNewLine();

        line.addLine(Eigen::Vector2d(_state.get("x"), _state.get("y")),
                     Eigen::Vector2d(_state.get("x") + 1.0 * std::cos(_state.get("psi")), _state.get("y") + 1.0 * std::sin(_state.get("psi"))));
        publisher.publish();
    }

    void AntbotPlanner::reset(bool success)
    {
        LOG_INFO("Resetting");
        boost::mutex::scoped_lock l(_reset_mutex);

        if (_reset_simulation_client->service_is_ready())
        {
            auto future = _reset_simulation_client->async_send_request(
                std::make_shared<std_srvs::srv::Empty::Request>(_reset_msg));
            if (_service_executor.spin_until_future_complete(future) != rclcpp::FutureReturnCode::SUCCESS)
                _reset_simulation_client->remove_pending_request(future);
        }
        if (_reset_ekf_client->service_is_ready())
        {
            auto future = _reset_ekf_client->async_send_request(
                std::make_shared<robot_localization::srv::SetPose::Request>(_reset_pose_msg));
            if (_service_executor.spin_until_future_complete(future) != rclcpp::FutureReturnCode::SUCCESS)
                _reset_ekf_client->remove_pending_request(future);
        }
        _reset_simulation_pub->publish(std_msgs::msg::Empty());

        _planner->reset(_state, _data, success);
        _data.costmap = &costmap_;

        get_clock()->sleep_for(rclcpp::Duration::from_seconds(
            1.0 / CONFIG["control_frequency"].as<double>()));

        done_ = false;

        _timeout_timer.start();
    }

    void AntbotPlanner::collisionCallback(const std_msgs::msg::Float64::ConstSharedPtr &msg)
    {
        LOG_MARK("Collision callback");

        _data.intrusion = (float)(msg->data);

        if (_data.intrusion > 0.)
            RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 500,
                                      "Collision detected (Intrusion: " << _data.intrusion << ")");
    }

    void AntbotPlanner::publishPose()
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = _state.get("x");
        pose.pose.position.y = _state.get("y");
        pose.pose.orientation = RosTools::angleToQuaternion(_state.get("psi"));

        pose.header.stamp = now();
        pose.header.frame_id = "map";

        _pose_pub->publish(pose);
    }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try
    {
        auto planner = std::make_shared<AntbotPlanner>();
        rclcpp::spin(planner);
    }
    catch (const std::exception &exception)
    {
        RCLCPP_ERROR(rclcpp::get_logger("antbot_planner"), "%s", exception.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
