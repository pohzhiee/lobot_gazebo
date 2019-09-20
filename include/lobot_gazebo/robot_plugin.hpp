#ifndef _ROBOT_PLUGIN_HPP
#define _ROBOT_PLUGIN_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64.hpp>

#include <rclcpp/rclcpp.hpp>
#include <parameter_server_interfaces/srv/get_all_joints.hpp>
#include <ros2_control_interfaces/msg/joint_commands.hpp>


namespace gazebo_plugins
{
class RobotPluginPrivate
{
public:

/**
 * @brief Callback to be called at every simulation iteration.
 * 
 * @param _info Updated simulation info.
 */
    void OnUpdate(const gazebo::common::UpdateInfo & _info);
    /// A pointer to the GazeboROS node.
    gazebo_ros::Node::SharedPtr ros_node_;


/**
 * @brief Connection to event called at every world iteration.
 * 
 */
    gazebo::event::ConnectionPtr update_connection_;

/**
 * @brief Pointer to joints
 * 
 */
    std::vector<gazebo::physics::JointPtr> joints_;

    /// Pointer to model.
    gazebo::physics::ModelPtr model_;

    /// Protect variables accessed on callbacks.
    std::mutex lock_;

/**
 * @brief Update period in seconds
 * 
 */
    double update_period_;

    /// Last update time.
    gazebo::common::Time last_update_time_;

    std::vector<double> commands_;

    rclcpp::Subscription<ros2_control_interfaces::msg::JointCommands>::SharedPtr cmd_subscription_;
};


class RobotPlugin : public gazebo::ModelPlugin
{
public:
    RobotPlugin();
public:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    void Reset() override;
private:
    /// Private data pointer
    std::unique_ptr<RobotPluginPrivate> impl_;
    /**
     * @brief Callback function whenever a new command is received. Updates command buffer
     * 
     * @param msg 
     */
    void CommandSubscriptionCallback(ros2_control_interfaces::msg::JointCommands::UniquePtr msg);
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(RobotPlugin)
} // namespace gazebo_plugins
#endif