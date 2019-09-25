// Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the company nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include "ros2_control_gazebo/my_joint_state_publisher.hpp"
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <parameter_server_interfaces/srv/get_all_joints.hpp>

#include <memory>
#include <string>
#include <vector>
#include <chrono>

namespace gazebo_plugins
{
class MyJointStatePublisherPrivate
{
public:
    /// Callback to be called at every simulation iteration.
    /// \param[in] info Updated simulation info.
    void OnUpdate(const gazebo::common::UpdateInfo &info);

    /// A pointer to the GazeboROS node.
    gazebo_ros::Node::SharedPtr ros_node_;

    /// Joint state publisher.
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    /// Joints being tracked.
    std::vector<gazebo::physics::JointPtr> joints_;

    /// Period in seconds
    double update_period_;

    /// Keep last time an update was published
    gazebo::common::Time last_update_time_;

    /// Pointer to the update event connection.
    gazebo::event::ConnectionPtr update_connection_;
};

MyJointStatePublisher::MyJointStatePublisher()
    : impl_(std::make_unique<MyJointStatePublisherPrivate>())
{
}

MyJointStatePublisher::~MyJointStatePublisher()
{
}

void MyJointStatePublisher::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    // ROS node
    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

    auto request_node = std::make_shared<rclcpp::Node>("robot_joint_state_plugin_request_node");
    auto client1 = request_node->create_client<parameter_server_interfaces::srv::GetAllJoints>("/GetAllControlJoints");
    using namespace std::chrono_literals;
    // Joints
    std::vector<std::string> joint_names;
    std::string robotName;
    // Get the robot_name from the plugin sdf
    if (sdf->HasElement("robot_name"))
    {
        sdf::ElementPtr robot_elem = sdf->GetElement("robot_name");
        robotName = robot_elem->Get<std::string>();
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Robot name from sdf: %s", robotName.c_str());
    }
    else
    {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Robot field not populated in sdf");
    }

    auto retryCount = 0;
    while (retryCount < 8)
    {
        client1->wait_for_service(1s);
        // Get all the joint names for the robot
        if (!client1->service_is_ready())
        {
            RCLCPP_ERROR(impl_->ros_node_->get_logger(), "GetAllJoints service failed to start, check that parameter server is launched");
            retryCount++;
            continue;
        }

        auto req = std::make_shared<parameter_server_interfaces::srv::GetAllJoints::Request>();
        req->robot = robotName;
        auto resp = client1->async_send_request(req);
        auto spin_status = rclcpp::spin_until_future_complete(request_node, resp, 5s);
        if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(impl_->ros_node_->get_logger(), "GetAllJoints service failed to execute (spin failed)");
            retryCount++;
            continue;
        }
        auto status = resp.wait_for(1s);
        if (status != std::future_status::ready)
        {
            RCLCPP_ERROR(impl_->ros_node_->get_logger(), "GetAllJoints service failed to execute");
            retryCount++;
            continue;
        }

        auto res = resp.get();
        joint_names = res->joints;
        break;
    }

    // Register all the joints based on the joint names from parameter server
    for (auto &joint_name : joint_names)
    {
        auto joint = model->GetJoint(joint_name);
        if (!joint)
        {
            RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint %s does not exist!", joint_name.c_str());
        }
        else
        {
            impl_->joints_.push_back(joint);
            RCLCPP_INFO(impl_->ros_node_->get_logger(), "Registering joint %s",
                        joint_name.c_str());
        }
    }

    // Update rate
    double update_rate = 100.0;
    if (!sdf->HasElement("update_rate"))
    {
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Missing <update_rate>, defaults to %f",
                    update_rate);
    }
    else
    {
        update_rate = sdf->GetElement("update_rate")->Get<double>();
    }

    if (update_rate > 0.0)
    {
        impl_->update_period_ = 1.0 / update_rate;
    }
    else
    {
        impl_->update_period_ = 0.0;
    }

    impl_->last_update_time_ = model->GetWorld()->SimTime();

    // Joint state publisher
    impl_->joint_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 1000);

    // Callback on every iteration
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&MyJointStatePublisherPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void MyJointStatePublisherPrivate::OnUpdate(const gazebo::common::UpdateInfo &info)
{
    gazebo::common::Time current_time = info.simTime;

    // If the world is reset, for example
    if (current_time < last_update_time_)
    {
        RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
        last_update_time_ = current_time;
    }

    // Check period
    double seconds_since_last_update = (current_time - last_update_time_).Double();

    if (seconds_since_last_update < update_period_)
    {
        return;
    }

    // Populate message
    sensor_msgs::msg::JointState joint_state;

    joint_state.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
    joint_state.name.resize(joints_.size());
    joint_state.position.resize(joints_.size());
    joint_state.velocity.resize(joints_.size());

    for (unsigned int i = 0; i < joints_.size(); ++i)
    {
        auto joint = joints_[i];
        double velocity = joint->GetVelocity(0);
        double position = joint->Position(0);
        joint_state.name[i] = joint->GetName();
        joint_state.position[i] = position;
        joint_state.velocity[i] = velocity;
    }

    // Publish
    joint_state_pub_->publish(joint_state);

    // Update time
    last_update_time_ = current_time;
}

GZ_REGISTER_MODEL_PLUGIN(MyJointStatePublisher)
} // namespace gazebo_plugins