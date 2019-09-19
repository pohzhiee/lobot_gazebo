#include "lobot_gazebo/robot_plugin.hpp"

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

    // Get all the joint names for the robot
    if (client1->service_is_ready())
    {
        auto req = std::make_shared<parameter_server_interfaces::srv::GetAllJoints::Request>();
        req->robot = robotName;
        auto resp = client1->async_send_request(req);
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Sending async request...");
        auto spin_status = rclcpp::spin_until_future_complete(request_node, resp, 5s);
        if (spin_status == rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            auto status = resp.wait_for(1s);
            if (status == std::future_status::ready)
            {
                auto res = resp.get();
                joint_names = res->joints;
                // for (auto &j : res->joints)
                // {
                //     RCLCPP_INFO(impl_->ros_node_->get_logger(), "Joint: %s", j.c_str());
                // }
            }
            else
            {
                RCLCPP_ERROR(impl_->ros_node_->get_logger(), "GetAllJoints service failed to execute");
            }
        }
        else
        {
            RCLCPP_ERROR(impl_->ros_node_->get_logger(), "GetAllJoints service failed to execute (spin failed)");
        }
    }
    else
    {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(), "GetAllJoints service failed to start, check that parameter server is launched");
    }

    // Register all the joints based on the joint names from parameter server
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

    // Create the joint subscriptions
    impl_->commands_.resize(impl_->joints_.size());
    if (impl_->joints_.size() != 0)
    {
        for (size_t i = 0; i < impl_->joints_.size(); i++)
        {
            auto topicName = "/" + robotName + "/" + impl_->joints_[i]->GetName() + "/cmd";
            auto fp = [this, i](std_msgs::msg::Float64::UniquePtr msg) {
                auto j = this->impl_->joints_[i];
                j->SetVelocity(0, msg->data);
                // this->impl_->commands_[i] = msg->data;
                // RCLCPP_INFO(this->impl_->ros_node_->get_logger(), "Joint %d updated with %f", i, msg->data);
            };

            auto sub = impl_->ros_node_->create_subscription<std_msgs::msg::Float64>(topicName, rclcpp::SensorDataQoS(), fp);
            impl_->joint_cmd_subscriptions_.push_back(sub);
        }
    }
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

    auto realTime = _info.realTime.Double();
    auto simTime = _info.simTime.Double();
    // Update time
    last_update_time_ = current_time;
    // RCLCPP_WARN(ros_node_->get_logger(),
    //             "[%f] Plugin update called", realTime);
}

} //end namespace gazebo_plugins