#ifndef _ROBOT_PLUGIN_HPP
#define _ROBOT_PLUGIN_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/empty.hpp>

#include <rclcpp/rclcpp.hpp>
#include <parameter_server_interfaces/srv/get_all_joints.hpp>
#include <parameter_server_interfaces/srv/get_controller_pid.hpp>
#include <ros2_control_interfaces/msg/joint_control.hpp>

#include <unordered_map>
#include <memory>

namespace gazebo_plugins
{
using Empty = std_srvs::srv::Empty;
class RobotPluginPrivate
{
public:
    void OnUpdate(const gazebo::common::UpdateInfo &_info);
    gazebo_ros::Node::SharedPtr ros_node_;

    gazebo::physics::ModelPtr model_;
    std::string model_name_;

    std::unordered_map<std::string, gazebo::physics::JointPtr> joints_map_ = {};
    std::unordered_map<std::string, std::shared_ptr<gazebo::physics::JointController>> joint_controllers_map_ = {};
    std::unordered_map<std::string, double> goal_map_ = {};
    std::unordered_map<std::string, double> command_map_ = {};

    double update_period_;
    gazebo::common::Time last_update_time_;
    gazebo::common::Time last_print_time_;
    gazebo::event::ConnectionPtr update_connection_;

    rclcpp::Subscription<ros2_control_interfaces::msg::JointControl>::SharedPtr cmd_subscription_;
    rclcpp::Service<Empty>::SharedPtr reset_service_;
    void CommandSubscriptionCallback(ros2_control_interfaces::msg::JointControl::UniquePtr msg);
    void ResetServiceCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<Empty::Request> request,
        const std::shared_ptr<Empty::Response> response);
};

class RobotPlugin : public gazebo::ModelPlugin
{
public:
    RobotPlugin();

public:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    void Reset() override;

private:
    std::shared_ptr<RobotPluginPrivate> impl_;
    // Helper functions
    std::vector<std::string> GetJoints(const std::string &robotName, rclcpp::Node::SharedPtr request_node);
    std::unordered_map<std::string, gazebo::common::PID> GetPidParameters(const std::string &robot_name, rclcpp::Node::SharedPtr request_node);
    void SetUpdateRate(sdf::ElementPtr _sdf);
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(RobotPlugin)
} // namespace gazebo_plugins
#endif