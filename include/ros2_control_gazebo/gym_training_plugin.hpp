//
// Created by pohzhiee on 22/10/19.
//

#ifndef ROS2_CONTROL_GAZEBO_GYM_TRAINING_PLUGIN_HPP
#define ROS2_CONTROL_GAZEBO_GYM_TRAINING_PLUGIN_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <ros2_control_interfaces/srv/get_current_sim_time.hpp>

namespace gazebo_plugins {
    using GetSimTime = ros2_control_interfaces::srv::GetCurrentSimTime;
    class GymTrainingPlugin : public gazebo::WorldPlugin {
    public:
        GymTrainingPlugin();

        void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;
        void Reset() override;

    private:
        gazebo_ros::Node::SharedPtr ros_node_;
        uint32_t update_period_{};
        gazebo::common::Time last_update_time_;
        gazebo::physics::WorldPtr world_ptr_;
        gazebo::event::ConnectionPtr update_connection_;
        rclcpp::Service<GetSimTime>::SharedPtr get_sim_time_srv_;
        void OnUpdate(const gazebo::common::UpdateInfo &info);
        void handle_GetSimTime(const std::shared_ptr<rmw_request_id_t> request_header,
                                     const std::shared_ptr<GetSimTime::Request> request,
                                     const std::shared_ptr<GetSimTime::Response> response);

        // Helper functions
        double getUpdateRate(const rclcpp::Node::SharedPtr& request_node);
    };
    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_WORLD_PLUGIN(GymTrainingPlugin)
} // namespace gazebo_plugins


#endif //ROS2_CONTROL_GAZEBO_GYM_TRAINING_PLUGIN_HPP
