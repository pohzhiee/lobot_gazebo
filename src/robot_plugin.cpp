#include "ros2_control_gazebo/robot_plugin.hpp"
#include "parameter_server_interfaces/srv/get_all_pid.hpp"

#include <functional>
#include <string>
#include <chrono>

namespace gazebo_plugins
{
using namespace std::chrono_literals;

RobotPlugin::RobotPlugin()
    : impl_(std::make_unique<RobotPluginPrivate>())
{
}

void RobotPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    impl_->model_ = _model;
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Plugin loading...");
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
        RCLCPP_FATAL(impl_->ros_node_->get_logger(), "Robot field not populated in sdf");
        return;
    }
    impl_->robot_name_ = robotName;
    // Get the joints from parameter server
    auto request_node = std::make_shared<rclcpp::Node>("robot_plugin_request_node");
    auto joint_names = GetJoints(robotName, request_node);
    auto pid_params_map = GetPidParameters(robotName, request_node);

    // Register the joints and the corresponding controllers
    for (auto &joint_name : joint_names)
    {
        auto joint = _model->GetJoint(joint_name);
        if (!joint)
        {
            RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint %s does not exist!", joint_name.c_str());
            continue;
        }
        impl_->joints_map_[joint_name] = joint;
        auto controller = std::make_shared<gazebo::physics::JointController>(_model);
        controller->AddJoint(joint);
        // Somehow after adding the joint to the controller, to set any pid or target of a joint it takes the robot name into account
        // So the fullJointName accounts for the robot name as well
        auto fullJointName = impl_->robot_name_ + "::" + joint_name;
        controller->SetPositionPID(fullJointName, std::move(pid_params_map[joint_name]));
        controller->SetPositionTarget(fullJointName, 0.0);
        impl_->joint_controllers_map_[joint_name] = std::move(controller);

        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Registering joint %s", joint_name.c_str());
    }

    // Create the joint subscription
    impl_->cmd_subscription_ = impl_->ros_node_->create_subscription<ros2_control_interfaces::msg::JointControl>(
        "/" + robotName + "/control", rclcpp::SensorDataQoS(),
        std::bind(&RobotPluginPrivate::CommandSubscriptionCallback, impl_, std::placeholders::_1));

    SetUpdateRate(_sdf);

    impl_->last_update_time_ = _model->GetWorld()->SimTime();

    //Create reset service
    auto reset_cb_fp = std::bind(&RobotPluginPrivate::ResetServiceCallback, impl_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    impl_->reset_service_ =
        impl_->ros_node_->create_service<Empty>("/" + robotName + "/reset", reset_cb_fp);

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
    // Update goal
    for (auto &pair : goal_map_)
    {
        auto joint_name = pair.first;
        auto goal = pair.second;
        joint_controllers_map_[joint_name]->SetPositionTarget(robot_name_ + "::" + joint_name, goal);
    }

    // Check period
    double seconds_since_last_update = (current_time - last_update_time_).Double();
    double seconds_since_last_print = (current_time - last_print_time_).Double();

    if (seconds_since_last_update < update_period_)
    {
        return;
    }
    constexpr auto print_period = 6.0;
    for (auto &pair : joint_controllers_map_)
    {
        auto jointName = pair.first;
        auto &controllerPtr = pair.second;
        // RCLCPP_WARN(ros_node_->get_logger(), "Controller updating %s", pair.first.c_str());
        auto vel = controllerPtr->GetVelocities();
        auto forces = controllerPtr->GetForces();
        auto pos = controllerPtr->GetPositions();
        auto pids = controllerPtr->GetPositionPIDs();
        controllerPtr->Update();
        if (seconds_since_last_print < print_period)
            continue;
        auto fullJointName = robot_name_ + "::" + jointName;
        RCLCPP_INFO(ros_node_->get_logger(), "Updating controller %s with cmd: %f", jointName.c_str(), pids[fullJointName].GetCmd());
    }

    if (seconds_since_last_print >= print_period)
    {
        RCLCPP_INFO(ros_node_->get_logger(), "======================");
        last_print_time_ = current_time;
    }
    // Update time
    last_update_time_ = current_time;
}

void RobotPluginPrivate::CommandSubscriptionCallback(ros2_control_interfaces::msg::JointControl::UniquePtr msg)
{
    // Uses map to write to buffer
    {
        auto msgNameSize = msg->joints.size();
        auto msgCmdSize = msg->goals.size();
        if (msgNameSize != msgCmdSize)
        {
            RCLCPP_ERROR_ONCE(ros_node_->get_logger(), "Message number of joints don't match number of commands");
        }

        for (size_t i = 0; i < msgNameSize; i++)
        {
            auto goal = msg->goals[i];
            auto name = msg->joints[i];
            auto jointPtr = joints_map_[name];
            goal_map_[name] = goal;
        }
    }
}

void RobotPluginPrivate::ResetServiceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<Empty::Request> request,
    const std::shared_ptr<Empty::Response> response)
{
    (void)request_header;
    (void)request;
    (void)response;
    for (auto &pair : joints_map_)
    {
        auto jointPtr = pair.second;
        jointPtr->SetPosition(0, 0.0);
        jointPtr->SetVelocity(0, 0.0);
        jointPtr->SetForce(0, 0.0);
    }
}

std::vector<std::string> RobotPlugin::GetJoints(const std::string &robotName, rclcpp::Node::SharedPtr request_node)
{
    auto client1 = request_node->create_client<parameter_server_interfaces::srv::GetAllJoints>("/GetAllControlJoints");
    using namespace std::chrono_literals;
    client1->wait_for_service(1s);
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
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Getting joints...");
        auto spin_status = rclcpp::spin_until_future_complete(request_node, resp, 1s);
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
        return res->joints;
    }
    RCLCPP_FATAL(impl_->ros_node_->get_logger(), "Unable to get joints for robot: %s", robotName.c_str());
    return {};
}

std::unordered_map<std::string, gazebo::common::PID> RobotPlugin::GetPidParameters(const std::string &robotName, rclcpp::Node::SharedPtr request_node)
{
    using GetAllPid = parameter_server_interfaces::srv::GetAllPid;

    auto map = std::unordered_map<std::string, gazebo::common::PID>();
    auto client = request_node->create_client<GetAllPid>("/GetAllPid");
    unsigned int retryCount = 0;
    constexpr unsigned int maxRetries = 10;
    while (retryCount < maxRetries)
    {
        client->wait_for_service(1.5s);
        if (!client->service_is_ready())
        {
            retryCount++;
            RCLCPP_ERROR(impl_->ros_node_->get_logger(),
                         "GetAllPid service failed to start, check that parameter server is launched, retries left: %d", maxRetries - retryCount);
            continue;
        }

        auto req = std::make_shared<GetAllPid::Request>();
        req->robot = robotName;
        auto resp = client->async_send_request(req);
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Getting PID parameters for robot %s...", robotName.c_str());
        auto spin_status = rclcpp::spin_until_future_complete(request_node, resp, 1s);
        if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            retryCount++;
            RCLCPP_ERROR(impl_->ros_node_->get_logger(), "GetAllPid service failed to execute (spin failed), retries left: %d", maxRetries - retryCount);
            continue;
        }
        auto status = resp.wait_for(0.2s);
        if (status != std::future_status::ready)
        {
            retryCount++;
            RCLCPP_ERROR(impl_->ros_node_->get_logger(), "GetAllPid service failed to execute, retries left: %d", maxRetries - retryCount);
            continue;
        }
        auto res = resp.get();
        for (size_t i = 0; i < res->joints.size(); i++)
        {
            auto joint = res->joints[i];
            auto gazeboPid = gazebo::common::PID();
            gazeboPid.SetPGain(res->p[i]);
            gazeboPid.SetIGain(res->i[i]);
            gazeboPid.SetDGain(res->d[i]);
            gazeboPid.SetIMin(res->i_min[i]);
            gazeboPid.SetIMax(res->i_max[i]);
            map[joint] = std::move(gazeboPid);
        }
        return map;
    }
    RCLCPP_FATAL(impl_->ros_node_->get_logger(), "Unable to get pid values for joints of robot: %s", robotName.c_str());
    return {};
}

void RobotPlugin::SetUpdateRate(sdf::ElementPtr _sdf)
{
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
}

} //end namespace gazebo_plugins