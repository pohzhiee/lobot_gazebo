#include "ros2_control_gazebo/robot_control_plugin.hpp"
#include "parameter_server_interfaces/srv/get_all_pid.hpp"
#include "py_wrappers/forward_kinematics.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <functional>
#include <string>
#include <chrono>
#include <algorithm>

namespace{
    std::unique_ptr<ForwardKinematics> fk_ptr;
    bool is_valid_joint_values(const std::vector<double> &joint_states){
        std::string lobot_desc_share_dir = ament_index_cpp::get_package_share_directory("lobot_description");
        std::string urdf_path = lobot_desc_share_dir + "/robots/arm_standalone.urdf";
        if(fk_ptr == nullptr){
            fk_ptr = std::make_unique<ForwardKinematics>(urdf_path);
        }
        auto pose = fk_ptr->calculate("world", "grip_end_point", joint_states);
        return !(pose.translation.z < 0);
    }
}


namespace gazebo_plugins {
    constexpr double back_emf_constant = 0.8;
    using namespace std::chrono_literals;

    RobotControlPlugin::RobotControlPlugin()
            : impl_(std::make_shared<RobotControlPluginPrivate>()) {
    }

    void RobotControlPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        impl_->model_ = _model;
        auto nodeOptions = rclcpp::NodeOptions();
        nodeOptions.start_parameter_services(false);
        impl_->ros_node_ = gazebo_ros::Node::CreateWithArgs(_sdf->Get<std::string>("name"), nodeOptions);

        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Plugin loading...");
        // Robot name is the name that is obtained from the urdf, used to query the parameter server
        std::string robotName;
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

        // Get the joints from parameter server
        auto request_node = std::make_shared<rclcpp::Node>("robot_plugin_request_node");
        auto joint_names = GetJoints(robotName, request_node);
        auto pid_params_map = GetPidParameters(robotName, request_node);

        // Register the joints, make sure the relevant vectors are empty first
        impl_->pid_vec_.clear();
        impl_->joints_vec_.clear();
        // Some joints from the parameter server may not exist in the robot
        uint8_t index = 0;
        for (auto &joint_name : joint_names) {
            auto joint = _model->GetJoint(joint_name);
            if (!joint) {
                RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint %s does not exist! Ignoring...",
                             joint_name.c_str());
                continue;
            }

            impl_->joint_index_map_[joint_name] = index;
            RCLCPP_INFO(impl_->ros_node_->get_logger(), "Registering joint %s with id %lu", joint_name.c_str(), index);
            auto pid = std::make_unique<gazebo::common::PID>(pid_params_map[joint_name]);
            impl_->pid_vec_.emplace_back(std::move(pid));
            impl_->joints_vec_.emplace_back(std::move(joint));
            ++index;
        }
        // index+1 gives the total number of registered joints, so we can initialise our vectors appropriately
        impl_->goal_vec_.resize(index + 1);
        impl_->command_vec_.resize(index + 1);

        // Create the joint control subscription
        auto qos_profile = rclcpp::QoS(20).reliable();
        impl_->cmd_subscription_ = impl_->ros_node_->create_subscription<ros2_control_interfaces::msg::JointControl>(
                "/" + robotName + "/control", qos_profile,
                std::bind(&RobotControlPluginPrivate::CommandSubscriptionCallback, impl_, std::placeholders::_1));

        // Create the set positions service
        auto randomPositionsCallback = [this](std::shared_ptr<rmw_request_id_t> a,
                                              std::shared_ptr<RandomPositions::Request> b,
                                              std::shared_ptr<RandomPositions::Response> c) {
            impl_->handle_RandomPositions(a, b, c);
        };
        impl_->random_positions_srv_ = impl_->ros_node_->create_service<RandomPositions>("/random_positions",
                                                                                         randomPositionsCallback);

        SetUpdateRate(_sdf);
        impl_->last_update_time_ = _model->GetWorld()->SimTime();

        impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                std::bind(&RobotControlPluginPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
    }

    void RobotControlPlugin::Reset() {
//        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Simulation reset called");
        impl_->Reset();
    }

    void RobotControlPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo &_info) {

        const gazebo::common::Time &current_time = _info.simTime;
        std::vector<double> local_goal_vec;
        {
            std::lock_guard<std::mutex> lock(goal_lock_);
            local_goal_vec = goal_vec_;
        }
        if (random_position) {
            random_position.store(false);
//            if(joints_vec_.size() != local_goal_vec.size()){
//                RCLCPP_ERROR(ros_node_->get_logger(), "Registered number of joints don't match randomised joints, "
//                                                      "joints_vec_size: %lu, goal_vec_size: %lu",
//                                                      joints_vec_.size(), local_goal_vec.size());
//            }
//            else{
            for (unsigned long i = 0; i < joints_vec_.size(); i++) {
                joints_vec_[i]->SetPosition(0, local_goal_vec[i]);
                RCLCPP_WARN(ros_node_->get_logger(),"Setting joint %s to %f", joints_vec_[i]->GetName().c_str(), local_goal_vec[i]);
            }
//            }
        }

        // If the world is reset, for example
        if (current_time < last_update_time_) {
            RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
            last_update_time_ = current_time;
            return;
        }

        // Check period
        auto time_since_last_update = current_time - last_update_time_;
        double seconds_since_last_update = time_since_last_update.Double();

        if (seconds_since_last_update < update_period_) {
            // Set the force to the previous controller command, otherwise it will default to 0
            UpdateForceFromCmdBuffer();
            return;
        }

        // Update the PID command if PID update period is reached for each joint
        for (auto &pair : joint_index_map_) {
            auto &joint_name = pair.first;
            auto joint_index = pair.second;
            auto &pid = pid_vec_[joint_index];
            auto joint = model_->GetJoint(joint_name);
            auto current_pos = joint->Position(0);
            auto error = current_pos - local_goal_vec[joint_index];
            double pid_output_command = pid->Update(error, time_since_last_update);
            command_vec_[joint_index] = pid_output_command;
        }
        UpdateForceFromCmdBuffer();

        // Update time
        last_update_time_ = current_time;
    }

    void RobotControlPluginPrivate::CommandSubscriptionCallback(
            ros2_control_interfaces::msg::JointControl::UniquePtr msg) {
        // Uses map to write to buffer
        {
            auto zero_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
            auto msg_time = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
            auto last_update_time = rclcpp::Time(last_update_time_.sec, last_update_time_.nsec, RCL_ROS_TIME);
            if (msg_time > zero_time) {
                if (msg_time < last_update_time) {
                    RCLCPP_WARN(ros_node_->get_logger(),
                                "Ignoring outdated message, Msg time: [%u][%lu], Last update time: [%u][%lu]",
                                msg->header.stamp.sec, msg->header.stamp.nanosec, last_update_time_.sec,
                                last_update_time_.nsec);
                    return;
                }
            }

            auto msgNameSize = msg->joints.size();
            auto msgCmdSize = msg->goals.size();
            if (msgNameSize != msgCmdSize) {
                RCLCPP_ERROR_ONCE(ros_node_->get_logger(), "Message number of joints don't match number of commands");
            }

            std::lock_guard<std::mutex> lock(goal_lock_);
            for (size_t i = 0; i < msgNameSize; i++) {
                auto goal = msg->goals[i];
                auto name = msg->joints[i];
                auto index_iter = joint_index_map_.find(name);
                if (index_iter == joint_index_map_.end()) {
                    RCLCPP_WARN_ONCE(ros_node_->get_logger(), "Message joint [%s] not registered in the plugin",
                                     name.c_str());
                    continue;
                }
                uint8_t index = (*index_iter).second;
                goal_vec_[index] = goal;
            }
        }
    }

    void RobotControlPluginPrivate::Reset() {
        for (const auto &joint : joints_vec_) {
            joint->SetPosition(0, 0.0);
            joint->SetVelocity(0, 0.0);
            joint->SetForce(0, 0.0);
        }
        for (const auto &pid : pid_vec_) {
            pid->Reset();
        }
        {
            std::lock_guard<std::mutex> lock(goal_lock_);
            for (auto &goal : goal_vec_) {
                goal = 0;
            }
        }
        last_update_time_ = gazebo::common::Time(0, 0);
    }

    void RobotControlPluginPrivate::handle_RandomPositions(const std::shared_ptr<rmw_request_id_t> &request_header,
                                                           const std::shared_ptr<RandomPositions::Request> &request,
                                                           const std::shared_ptr<RandomPositions::Response> &response) {
        (void) request_header;
        (void) request;
        {
            std::lock_guard<std::mutex> lock(goal_lock_);
            bool valid = false;
            do{
                response->joints.clear();
                response->positions.clear();
                response->joints.resize(joint_index_map_.size());
                response->positions.resize(joint_index_map_.size());
                for (auto &pair: joint_index_map_) {
                    auto index = pair.second;
                    auto &joint = joints_vec_[index];
                    auto &pid = pid_vec_[index];

                    auto upper_limit = joint->UpperLimit(0);
                    auto lower_limit = joint->LowerLimit(0);
                    std::uniform_real_distribution<double> dist(lower_limit, upper_limit);
                    auto rand_angle = dist(rd_);
                    joint->SetPosition(0, rand_angle);
                    joint->SetVelocity(0, 0.0);
                    joint->SetForce(0, 0.0);
                    pid->Reset();
                    goal_vec_[index] = rand_angle;
                    response->joints[index] = (joint->GetName());
                    response->positions[index] = (rand_angle);
                    RCLCPP_INFO(ros_node_->get_logger(), "Joint %s with val %f", joint->GetName().c_str(), rand_angle);
                }
                valid = is_valid_joint_values(response->positions);
            } while(!valid);
            random_position.store(true);
        }
    }

    void RobotControlPluginPrivate::UpdateForceFromCmdBuffer() {
        for (auto &pair : joint_index_map_) {
            auto &joint_name = pair.first;
            auto joint_index = pair.second;
            auto command = command_vec_[joint_index];
            auto joint = model_->GetJoint(joint_name);
            if (joint != nullptr) {
                // It seems like the GetVelocity method is subjected to a very high rounding error
                // It rounds once during the current_pos - previous_pos
                // It rounds another time during the division by time step
                // The smaller the time step, the larger the floating point error is
                // That is why the velocity seemingly never goes to a very small value
                auto current_vel = joint->GetVelocity(0);

                auto max_torque = joint->GetEffortLimit(0);
                auto clamped_output = std::clamp(command, -max_torque, max_torque);
                auto desired_output_torque = clamped_output - back_emf_constant * current_vel;
                auto current_force = joint->GetForce(0);
                // Gazebo is strange, even when you get a positive value for force, if you didn't call set force it wouldn't actually apply the force
                // However, when you call SetForce(), it merely adds your passed force value to the existing value and apply that force instead
                // Therefore, we need to reduce the entire amount of force obtained by GetForce(), such that SetForce() applies the force that we want
                // instead of adding on to the old one
                // This is separated into two steps because there is a clamping mechanism in set force, so we can't combine the values and set
                joint->SetForce(0, -current_force);
                joint->SetForce(0, desired_output_torque);
                //Debugging stuff
/*
                if (debug && joint_name == "arm_1_joint") {
                    auto current_pos = joint->Position(0);
                    auto force = joint->GetForce(0);
                    auto error = current_pos - local_goal_vec[joint_index];
                    const std::unique_ptr<gazebo::common::PID> &pid_ptr = pid_vec_[joint_index];
                    double pError, iError, dError;
                    pid_ptr->GetErrors(pError, iError, dError);
                    auto dTerm = pid_ptr->GetDGain() * dError;
                    auto iTerm = pid_ptr->GetIGain() * iError;
                    auto pTerm = pid_ptr->GetPGain() * pError;
                    RCLCPP_WARN(ros_node_->get_logger(),
                                "[%d][%09d] Command: %f\t Set Force: %f\t Desired Force: %f\t Ori Vel: %f\t Error: %f",
                                current_time.sec, current_time.nsec, command, force, desired_output_torque, current_vel, error);
                    RCLCPP_WARN(ros_node_->get_logger(),
                                "[%d][%09d] Command: %f\t P: %f\t I: %f\t D: %f",
                                current_time.sec, current_time.nsec, command, pTerm, iTerm, dTerm);
                }*/

            } else {
                RCLCPP_WARN(ros_node_->get_logger(), "Joint [%s] not found", joint_name.c_str());
            }
        }
    }

    std::vector<std::string>
    RobotControlPlugin::GetJoints(const std::string &robotName, rclcpp::Node::SharedPtr request_node) {
        auto client1 = request_node->create_client<parameter_server_interfaces::srv::GetAllJoints>(
                "/GetAllControlJoints");
        using namespace std::chrono_literals;
        client1->wait_for_service(1s);
        auto retryCount = 0;
        while (retryCount < 8) {
            client1->wait_for_service(1s);
            if (!client1->service_is_ready()) {
                RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                             "GetAllJoints service failed to start, check that parameter server is launched");
                retryCount++;
                continue;
            }

            auto req = std::make_shared<parameter_server_interfaces::srv::GetAllJoints::Request>();
            req->robot = robotName;
            auto resp = client1->async_send_request(req);
            RCLCPP_INFO(impl_->ros_node_->get_logger(), "Getting joints...");
            auto spin_status = rclcpp::spin_until_future_complete(request_node, resp, 1s);
            if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(impl_->ros_node_->get_logger(), "GetAllJoints service failed to execute (spin failed)");
                retryCount++;
                continue;
            }
            auto status = resp.wait_for(1s);
            if (status != std::future_status::ready) {
                RCLCPP_ERROR(impl_->ros_node_->get_logger(), "GetAllJoints service failed to execute");
                retryCount++;
                continue;
            }
            auto res = resp.get();
            return res->joints;
        }
        RCLCPP_FATAL(impl_->ros_node_->get_logger(), "Unable to get joints for robot: %s", robotName.c_str());
        return {};
    }

    std::unordered_map<std::string, gazebo::common::PID>
    RobotControlPlugin::GetPidParameters(const std::string &robotName, rclcpp::Node::SharedPtr request_node) {
        using GetAllPid = parameter_server_interfaces::srv::GetAllPid;

        auto map = std::unordered_map<std::string, gazebo::common::PID>();
        auto client = request_node->create_client<GetAllPid>("/GetAllPid");
        unsigned int retryCount = 0;
        constexpr unsigned int maxRetries = 10;
        while (retryCount < maxRetries) {
            client->wait_for_service(1.5s);
            if (!client->service_is_ready()) {
                retryCount++;
                RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                             "GetAllPid service failed to start, check that parameter server is launched, retries left: %d",
                             maxRetries - retryCount);
                continue;
            }

            auto req = std::make_shared<GetAllPid::Request>();
            req->robot = robotName;
            auto resp = client->async_send_request(req);
            RCLCPP_INFO(impl_->ros_node_->get_logger(), "Getting PID parameters for robot %s...", robotName.c_str());
            auto spin_status = rclcpp::spin_until_future_complete(request_node, resp, 1s);
            if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS) {
                retryCount++;
                RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                             "GetAllPid service failed to execute (spin failed), retries left: %d",
                             maxRetries - retryCount);
                continue;
            }
            auto status = resp.wait_for(0.2s);
            if (status != std::future_status::ready) {
                retryCount++;
                RCLCPP_ERROR(impl_->ros_node_->get_logger(), "GetAllPid service failed to execute, retries left: %d",
                             maxRetries - retryCount);
                continue;
            }
            auto res = resp.get();
            for (size_t i = 0; i < res->joints.size(); i++) {
                auto joint = res->joints[i];
                auto gazeboPid = gazebo::common::PID();
                gazeboPid.SetPGain(res->p[i]);
                gazeboPid.SetIGain(res->i[i]);
                gazeboPid.SetDGain(res->d[i]);
                gazeboPid.SetIMin(res->i_min[i]);
                gazeboPid.SetIMax(res->i_max[i]);
                map[joint] = gazeboPid;
            }
            return map;
        }
        RCLCPP_FATAL(impl_->ros_node_->get_logger(), "Unable to get pid values for joints of robot: %s",
                     robotName.c_str());
        return {};
    }

    void RobotControlPlugin::SetUpdateRate(sdf::ElementPtr _sdf) {
        double update_rate = 100.0;
        if (!_sdf->HasElement("update_rate")) {
            RCLCPP_INFO(impl_->ros_node_->get_logger(), "Missing <update_rate>, defaults to %f",
                        update_rate);
        } else {
            update_rate = _sdf->GetElement("update_rate")->Get<double>();
        }

        if (update_rate > 0.0) {
            impl_->update_period_ = 1.0 / update_rate;
        } else {
            impl_->update_period_ = 0.0;
        }
    }

} //end namespace gazebo_plugins