#include "ros2_control_gazebo/robot_plugin.hpp"

#include <functional>
#include <string>

namespace gazebo_plugins
{
RobotPlugin::RobotPlugin()
    : impl_(std::make_unique<RobotPluginPrivate>())
{
}

void RobotPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    impl_->model_ = _model;
    impl_->joints_ = {};
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
    auto request_node = std::make_shared<rclcpp::Node>("robot_plugin_request_node");
    auto client1 = request_node->create_client<parameter_server_interfaces::srv::GetAllJoints>("/GetAllControlJoints");
    using namespace std::chrono_literals;
    client1->wait_for_service(1s);
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Plugin loading...");
    std::vector<std::string> joint_names;
    std::string robotName;
    // Get the robot_name from the plugin sdf
    if (_sdf->HasElement("robot_name"))
    {
        sdf::ElementPtr robot_elem = _sdf->GetElement("robot_name");
        robotName = robot_elem->Get<std::string>();
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Robot name from sdf: %s", robotName.c_str());
    }
    else
    {
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Robot field not populated in sdf");
    }

    // Get the joints from parameter server
    auto retryCount = 0;
    while (retryCount < 8)
    {
        client1->wait_for_service(1s);
        if (!client1->service_is_ready())
        {
            RCLCPP_ERROR(impl_->ros_node_->get_logger(), "GetAllJoints service failed to start, check that parameter server is launched");
            retryCount++;
            continue;
        }

        auto req = std::make_shared<parameter_server_interfaces::srv::GetAllJoints::Request>();
        req->robot = robotName;
        auto resp = client1->async_send_request(req);
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Sending async request...");
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

    for (auto &joint_name : joint_names)
    {
        auto joint = _model->GetJoint(joint_name);
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
    std::string node_name = _sdf->Get<std::string>("name");
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Plugin name %s\n", node_name.c_str());

    // Create the joint subscription
    impl_->commands_.resize(impl_->joints_.size());
    impl_->cmd_subscription_ = impl_->ros_node_->create_subscription<ros2_control_interfaces::msg::JointCommands>(
        "/" + robotName + "/sim/cmd", rclcpp::SensorDataQoS(),
        std::bind(&RobotPlugin::CommandSubscriptionCallback, this, std::placeholders::_1));

    // Set update rate stuff
    // Update rate
    double update_rate = 100.0;
    if (!_sdf->HasElement("update_rate"))
    {
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Missing <update_rate>, defaults to %f",
                    update_rate);
    }
    else
    {
        update_rate = _sdf->GetElement("update_rate")->Get<double>();
    }

    if (update_rate > 0.0)
    {
        impl_->update_period_ = 1.0 / update_rate;
    }
    else
    {
        impl_->update_period_ = 0.0;
    }

    impl_->last_update_time_ = _model->GetWorld()->SimTime();

    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&RobotPluginPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
} // namespace gazebo_plugins

void RobotPlugin::Reset()
{
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Simulation reset called");
}

void RobotPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo &_info)
{

    gazebo::common::Time current_time = _info.simTime;

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

    //  for (size_t i = 0; i < joints_.size(); i++)
    // {
    //     joints_[i]->SetForce(0, commands_[i]);
    // }

    // Update time
    last_update_time_ = current_time;
}

void RobotPlugin::CommandSubscriptionCallback(ros2_control_interfaces::msg::JointCommands::UniquePtr msg)
{
    // Safe method which writes directly to JointPtr
    {
        auto robotJointSize = impl_->joints_.size();
        auto inputCmdSize = msg->commands.size();
        if (robotJointSize == inputCmdSize)
        {
            for (size_t i = 0; i < inputCmdSize; i++)
            {
                auto robotJoints = impl_->joints_;
                auto msgJoints = msg->joint_names;
                auto jointName = msgJoints[i];
                auto fp = [&jointName](const gazebo::physics::JointPtr &jointPtr) -> bool { return jointName.compare(jointPtr->GetName()) == 0; };
                auto joint_iter = std::find_if(robotJoints.cbegin(), robotJoints.cend(), fp);
                if (joint_iter != robotJoints.cend())
                {
                    auto robotJoint = *joint_iter;
                    robotJoint->SetForce(0, msg->commands[i]);
                }
            }
        }
        else if (robotJointSize < inputCmdSize)
        {
            RCLCPP_WARN_ONCE(impl_->ros_node_->get_logger(), "Receiving more input than joints in robot, truncating...");
            for (size_t i = 0; i < robotJointSize; i++)
            {
                impl_->joints_[i]->SetForce(0, msg->commands[i]);
            }
        }
        else
        {
            RCLCPP_WARN_ONCE(impl_->ros_node_->get_logger(), "Receiving less input than joints in robot, some joints not controlled...");
            for (size_t i = 0; i < inputCmdSize; i++)
            {
                impl_->joints_[i]->SetForce(0, msg->commands[i]);
            }
        }
    }

    // Unsafe method which uses buffer, when using this method go to OnUpdate and uncomment the loop to set force based on buffer
    {
        /*
        auto robotCmdSize = impl_->commands_.size();
        auto inputCmdSize = msg->commands.size();
        if (robotCmdSize == inputCmdSize)
        {
            impl_->commands_ = msg->commands;
        }
        else if (robotCmdSize < inputCmdSize)
        {
            RCLCPP_WARN_ONCE(impl_->ros_node_->get_logger(), "Receiving more input than joints in robot, truncating...");
            for (size_t i = 0; i < robotCmdSize; i++)
            {
                impl_->commands_[i] = msg->commands[i];
            }
        }
        else
        {
            RCLCPP_WARN_ONCE(impl_->ros_node_->get_logger(), "Receiving less input than joints in robot, some joints not controlled...");
            for (size_t i = 0; i < inputCmdSize; i++)
            {
                impl_->commands_[i] = msg->commands[i];
            }
        }
    */
    }

    // Method with safer value assign but longer computation time, now commented
    { /*
        auto inputNameSize = msg->joint_names.size();
        auto inputCmdSize = msg->commands.size();
        auto robotCmdSize = impl_->commands_.size();
        // Ignore joint names if size dont match with commands, basically runs the unsafe assignment method found above
        if (inputNameSize != inputCmdSize)
        {
            RCLCPP_ERROR_ONCE(impl_->ros_node_->get_logger(), "Input message has uneven lengths of joint names and commands, ignoring names");
            if (robotCmdSize == inputCmdSize)
            {
                impl_->commands_ = msg->commands;
            }
            else if (robotCmdSize < inputCmdSize)
            {
                RCLCPP_WARN_ONCE(impl_->ros_node_->get_logger(), "Receiving more input than joints in robot, truncating...");
                for (size_t i = 0; i < robotCmdSize; i++)
                {
                    impl_->commands_[i] = msg->commands[i];
                }
            }
            else
            {
                RCLCPP_WARN_ONCE(impl_->ros_node_->get_logger(), "Receiving less input than joints in robot, some joints not controlled...");
                for (size_t i = 0; i < inputCmdSize; i++)
                {
                    impl_->commands_[i] = msg->commands[i];
                }
            }
        }
        else
        {
            for (size_t i = 0; i < inputNameSize; i++)
            {
                auto inputJointName = msg->joint_names[i];
                for (size_t j = 0; j < impl_->joints_.size(); j++)
                {
                    auto joint = impl_->joints_[j];
                    // Check that the input joint name matches with any registered joints, if matches write the command
                    if (joint->GetName().compare(inputJointName) == 0)
                    {
                        impl_->commands_[j] = msg->commands[i];
                        break;
                    }
                }
            }
        }
    */
    }
}

} //end namespace gazebo_plugins