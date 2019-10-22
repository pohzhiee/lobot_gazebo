//
// Created by pohzhiee on 22/10/19.
//

#ifndef ROS2_CONTROL_GAZEBO_GYM_TRAINING_PLUGIN_HPP
#define ROS2_CONTROL_GAZEBO_GYM_TRAINING_PLUGIN_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

namespace gazebo_plugins {
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
        void OnUpdate(const gazebo::common::UpdateInfo &info);

        // Helper functions
        double getUpdateRate(const rclcpp::Node::SharedPtr& request_node);
    };
    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_WORLD_PLUGIN(GymTrainingPlugin)
} // namespace gazebo_plugins


#endif //ROS2_CONTROL_GAZEBO_GYM_TRAINING_PLUGIN_HPP
