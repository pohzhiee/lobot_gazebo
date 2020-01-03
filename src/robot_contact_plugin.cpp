//
// Created by pohzhiee on 29/10/19.
//

#include "ros2_control_gazebo/robot_contact_plugin.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "gazebo_ros/conversions/gazebo_msgs.hpp"
#include "gazebo_ros/conversions/builtin_interfaces.hpp"

namespace gazebo_plugins {

    RobotContactPlugin::RobotContactPlugin() : impl_(std::make_shared<RobotContactPluginPrivate>()) {

    }

    void RobotContactPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {

        impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                std::bind(&RobotContactPluginPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
        impl_->model_ = _model;
        auto nodeOptions = rclcpp::NodeOptions();
        nodeOptions.start_parameter_services(false);
        impl_->ros_node_ = gazebo_ros::Node::CreateWithArgs(_sdf->Get<std::string>("name"), nodeOptions);

        std::string robotName;
        std::string modelName;
        modelName = _model->GetName();
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Model name: %s", modelName.c_str());
        // Get the robot_name from the plugin sdf
        if (_sdf->HasElement("robot_name")) {
            sdf::ElementPtr robot_elem = _sdf->GetElement("robot_name");
            robotName = robot_elem->Get<std::string>();
            RCLCPP_INFO(impl_->ros_node_->get_logger(), "Robot name from sdf: %s", robotName.c_str());
        } else {
            RCLCPP_FATAL(impl_->ros_node_->get_logger(),
                         "Robot name field not populated in sdf. Please set element robot_name");
            return;
        }

        auto qos_profile = rclcpp::QoS(20).reliable();
        impl_->contact_publisher_ = impl_->ros_node_->create_publisher<gazebo_msgs::msg::ContactsState>(
                "/" + robotName + "/contacts", qos_profile);

        auto world_ptr = _model->GetWorld();
        auto physics_engine_ptr = world_ptr->Physics();
        auto contact_manager = physics_engine_ptr->GetContactManager();
        contact_manager->SetNeverDropContacts(true);
        impl_->contact_manager_ = contact_manager;
    }

    void RobotContactPlugin::Reset() {
        impl_->contact_manager_->Clear();
        impl_->contact_manager_->ResetCount();
    }

    void RobotContactPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo &_info) {

        const gazebo::common::Time &time = _info.simTime;
        auto contact_count = contact_manager_->GetContactCount();
        if (contact_count == 0)
            return;

        gazebo::msgs::Contacts contacts_gz_msg;

        auto contacts = contact_manager_->GetContacts();
        for (size_t i = 0; i < contact_count; i++) {
            gazebo::msgs::Contact *contactMsg = contacts_gz_msg.add_contact();
            contacts[i]->FillMsg(*contactMsg);
        }

        auto contact_state_msg = gazebo_ros::Convert<gazebo_msgs::msg::ContactsState>(contacts_gz_msg);
        auto time_msg = gazebo_ros::Convert<builtin_interfaces::msg::Time>(time);
        contact_state_msg.header.stamp = time_msg;
        contact_publisher_->publish(contact_state_msg);
    }
}