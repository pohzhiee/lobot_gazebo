#ifndef _ROBOT_PLUGIN_HPP
#define _ROBOT_PLUGIN_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64.hpp>

#include <rclcpp/rclcpp.hpp>
#include <parameter_server_interfaces/srv/get_all_joints.hpp>
#include <parameter_server_interfaces/srv/get_controller_pid.hpp>
#include <ros2_control_interfaces/msg/joint_control.hpp>
#include <ros2_control_interfaces/srv/random_positions.hpp>

#include <unordered_map>
#include <memory>
#include <mutex>
#include <random>

namespace gazebo_plugins
{
using RandomPositions = ros2_control_interfaces::srv::RandomPositions;
class RobotControlPluginPrivate
{
public:
    void OnUpdate(const gazebo::common::UpdateInfo &_info);
    rclcpp::Node::SharedPtr ros_node_;

    gazebo::physics::ModelPtr model_;

    gazebo::event::ConnectionPtr update_connection_;
    double update_period_;
    gazebo::common::Time last_update_time_;

/** 
 * The joint index map is because I needed a safe way to guarantee that I am accessing what I am supposed to,
 * regardless of the order of joints being sent in during runtime
 * Instead of having a hash table for every single mapping, which performs a hash function every lookup,
 * It is made more efficient by only hashing once and then getting the index, and then using the index to 
 * perform "unsafe" lookup of the vector elements
 */
    std::unordered_map<std::string, uint8_t> joint_index_map_ = {};
    std::vector<gazebo::physics::JointPtr> joints_vec_ = {};
    std::vector<std::unique_ptr<gazebo::common::PID>> pid_vec_ = {};
    std::vector<double> goal_vec_ = {};
    std::vector<double> command_vec_ = {};

    rclcpp::Subscription<ros2_control_interfaces::msg::JointControl>::SharedPtr cmd_subscription_;
    void CommandSubscriptionCallback(ros2_control_interfaces::msg::JointControl::UniquePtr msg);

    rclcpp::Service<RandomPositions>::SharedPtr random_positions_srv_;
    void handle_RandomPositions(const std::shared_ptr<rmw_request_id_t> &request_header,
                             const std::shared_ptr<RandomPositions::Request> &request,
                             const std::shared_ptr<RandomPositions::Response> &response);
    void Reset();
private:
    void UpdateForceFromCmdBuffer();
    std::mutex goal_lock_;

    std::random_device rd_;

    std::atomic_bool random_position{false};

};

class RobotControlPlugin : public gazebo::ModelPlugin
{
public:
    RobotControlPlugin();

public:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    void Reset() override;

private:
    std::shared_ptr<RobotControlPluginPrivate> impl_;
    // Helper functions
    std::vector<std::string> GetJoints(const std::string &robotName, rclcpp::Node::SharedPtr request_node);
    std::unordered_map<std::string, gazebo::common::PID> GetPidParameters(const std::string &robot_name, rclcpp::Node::SharedPtr request_node);
    void SetUpdateRate(sdf::ElementPtr _sdf);
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(RobotControlPlugin)
} // namespace gazebo_plugins
#endif