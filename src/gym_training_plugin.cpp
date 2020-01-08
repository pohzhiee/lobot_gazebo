//
// Created by pohzhiee on 22/10/19.
//

#include <chrono>
#include <functional>

#include "ros2_control_gazebo/gym_training_plugin.hpp"
#include "parameter_server_interfaces/srv/get_gym_update_rate.hpp"

namespace gazebo_plugins{
    using namespace std::chrono_literals;
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    GymTrainingPlugin::GymTrainingPlugin()= default;

    void GymTrainingPlugin::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) {
        auto nodeOptions = rclcpp::NodeOptions();
        nodeOptions.start_parameter_services(false);
        ros_node_ = gazebo_ros::Node::CreateWithArgs(_sdf->Get<std::string>("name"), nodeOptions);
        world_ptr_ = _world;
        // use lambda instead of bind, see here https://stackoverflow.com/questions/17363003/why-use-stdbind-over-lambdas-in-c14
        auto getSimTimeCallback = [this](std::shared_ptr<rmw_request_id_t> a,
                std::shared_ptr<GetSimTime::Request> b,
                std::shared_ptr<GetSimTime::Response> c){this->handle_GetSimTime(a, b, c);};
        auto createMarkerCallback = [this](std::shared_ptr<rmw_request_id_t> a,
                std::shared_ptr<CreateMarker::Request> b,
                std::shared_ptr<CreateMarker::Response> c ){
            this->handle_CreateMarker(a, b, c); };
        get_sim_time_srv_ = ros_node_->create_service<GetSimTime>("/get_current_sim_time", getSimTimeCallback);
        mode_model_srv_ = ros_node_->create_service<CreateMarker>("/create_marker", createMarkerCallback);
        RCLCPP_INFO_ONCE(ros_node_->get_logger(), "GymTrainingPlugin loaded");
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
        RCLCPP_INFO(ros_node_->get_logger(), "Update period set to: %lu ns", update_period_);
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
            RCLCPP_INFO(ros_node_->get_logger(), "Getting gym update rate...");
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
        const gazebo::common::Time &current_time = info.simTime;

        // If the world is reset, for example
        if (current_time < last_update_time_){
            // RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
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

    void GymTrainingPlugin::handle_GetSimTime(const std::shared_ptr<rmw_request_id_t> &request_header,
                           const std::shared_ptr<GetSimTime::Request> &request,
                           const std::shared_ptr<GetSimTime::Response> &response){
        (void) request_header;
        (void) request;
        auto time = world_ptr_->SimTime();
        response->sec = time.sec;
        response->nanosec = time.nsec;
    }

    void GymTrainingPlugin::handle_CreateMarker(const std::shared_ptr<rmw_request_id_t> &request_header,
                                                const std::shared_ptr<CreateMarker::Request> &request,
                                                const std::shared_ptr<CreateMarker::Response> &response){
        (void) request_header;
        // Most likely not thread safe to mess with the models outside of the physics thread
        // Ensure that this callback do not run when the physics thread is also running (i.e. simulation is unpaused)
        // http://answers.gazebosim.org/question/21585/cannot-delete-models-when-using-the-transport-system/

        auto new_pose = ignition::math::Pose3d(request->x, request->y, request->z, request->roll, request->pitch, request->yaw);
        auto diameter = request->diameter;
        ignition::transport::Node node;

        std::string topicName = "/marker";

        // Publish to a Gazebo topic
        auto pub = node.Advertise<ignition::msgs::Marker>(topicName);

        // Create the marker message
        ignition::msgs::Marker markerMsg;
        markerMsg.set_ns("default");
        markerMsg.set_id(request->id);
        markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
        markerMsg.set_type(ignition::msgs::Marker::SPHERE);
        auto mat_ptr = markerMsg.mutable_material();
        ignition::msgs::Set(mat_ptr->mutable_ambient(), ignition::math::Color(0.1, 0.8, 0.85, 1.0));
        ignition::msgs::Set(mat_ptr->mutable_diffuse(), ignition::math::Color(0.1, 0.9, 0.9, 1.0));
        ignition::msgs::Set(mat_ptr->mutable_specular(), ignition::math::Color(0.1, 1.0, 0.8, 1.0));
        ignition::msgs::Set(mat_ptr->mutable_emissive(), ignition::math::Color(0.0, 0.0, 0.0, 0.0));
        ignition::msgs::Set(markerMsg.mutable_scale(), ignition::math::Vector3d(diameter, diameter, diameter));
        ignition::msgs::Set(markerMsg.mutable_pose(), new_pose);
        node.Request(topicName, markerMsg);

        response->status_message = "Success";
        response->success = true;
    }

}
