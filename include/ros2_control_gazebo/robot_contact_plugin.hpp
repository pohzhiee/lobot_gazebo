//
// Created by pohzhiee on 29/10/19.
//

#ifndef ROS2_CONTROL_GAZEBO_ROBOT_CONTACT_PLUGIN_HPP
#define ROS2_CONTROL_GAZEBO_ROBOT_CONTACT_PLUGIN_HPP

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>

namespace gazebo_plugins {

    class RobotContactPluginPrivate
    {
    public:
        void OnUpdate(const gazebo::common::UpdateInfo &_info);
        gazebo_ros::Node::SharedPtr ros_node_;

        gazebo::physics::ModelPtr model_;

        gazebo::event::ConnectionPtr update_connection_;

        gazebo::physics::ContactManager* contact_manager_;

        rclcpp::Publisher<gazebo_msgs::msg::ContactsState >::SharedPtr contact_publisher_;
    };

    class RobotContactPlugin : public gazebo::ModelPlugin {
    public:
        RobotContactPlugin();

        ~RobotContactPlugin() override = default;

        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
        void Reset() override;
    private:
        std::shared_ptr<RobotContactPluginPrivate> impl_;

    };
// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(RobotContactPlugin)
} // namespace gazebo_plugins

#endif //ROS2_CONTROL_GAZEBO_ROBOT_CONTACT_PLUGIN_HPP
