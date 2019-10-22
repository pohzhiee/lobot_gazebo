//
// Created by pohzhiee on 22/10/19.
//

#include <chrono>

#include "ros2_control_gazebo/gym_training_plugin.hpp"
#include "parameter_server_interfaces/srv/get_gym_update_rate.hpp"

namespace gazebo_plugins{
    using namespace std::chrono_literals;
    GymTrainingPlugin::GymTrainingPlugin()= default;

    void GymTrainingPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) {
        ros_node_ = gazebo_ros::Node::Get(_sdf);
        std::cerr << "LOAD!!!";
        world_ptr_ = _world;
        RCLCPP_WARN_ONCE(ros_node_->get_logger(), "GymTrainingPlugin loaded!!!!");
        auto request_node = std::make_shared<rclcpp::Node>("robot_joint_state_plugin_request_node");
        auto update_rate = getUpdateRate(request_node);

        if (update_rate > 0)
        {
            update_period_ = 1000000000 / update_rate;
        }
        else
        {
            update_period_ = 0;
        }
        RCLCPP_WARN_ONCE(ros_node_->get_logger(), "Update period set to: %lu", update_period_);
        update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                std::bind(&GymTrainingPlugin::OnUpdate, this, std::placeholders::_1));
    }

    void GymTrainingPlugin::Reset() {
        WorldPlugin::Reset();
    }

    double GymTrainingPlugin::getUpdateRate(const rclcpp::Node::SharedPtr& request_node){
        auto client1 = request_node->create_client<parameter_server_interfaces::srv::GetGymUpdateRate>("/GetGymUpdateRate");
        uint8_t retryCount = 0;
        while (retryCount < 20)
        {
            client1->wait_for_service(1s);
            if (!client1->service_is_ready())
            {
                RCLCPP_ERROR(ros_node_->get_logger(), "GetGymUpdateRate service failed to start, check that parameter server is launched");
                retryCount++;
                continue;
            }

            auto req = std::make_shared<parameter_server_interfaces::srv::GetGymUpdateRate::Request>();
            auto resp = client1->async_send_request(req);
            RCLCPP_INFO(ros_node_->get_logger(), "Getting joints...");
            auto spin_status = rclcpp::spin_until_future_complete(request_node, resp, 1s);
            if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(ros_node_->get_logger(), "GetGymUpdateRate service failed to execute (spin failed)");
                retryCount++;
                continue;
            }
            auto status = resp.wait_for(1s);
            if (status != std::future_status::ready)
            {
                RCLCPP_ERROR(ros_node_->get_logger(), "GetGymUpdateRate service failed to execute");
                retryCount++;
                continue;
            }
            auto res = resp.get();
            return res->update_rate;
        }
        return -1;
    }

    void GymTrainingPlugin::OnUpdate(const gazebo::common::UpdateInfo &info) {
        gazebo::common::Time current_time = info.simTime;

        // If the world is reset, for example
        if (current_time < last_update_time_){
            RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
            last_update_time_ = current_time;
        }

        // Check period
        uint32_t nsec_since_last_update = (current_time.nsec - last_update_time_.nsec) + (current_time.sec - last_update_time_.sec)*1000000000;
        if (nsec_since_last_update < update_period_)
        {
            return;
        }
        // Update time
        last_update_time_ = current_time;

        world_ptr_->SetPaused(true);
    }
}