cmake_minimum_required(VERSION 3.5)
project(ros2_control_gazebo)


# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

add_library(robot_control_plugin SHARED src/robot_control_plugin.cpp)
add_library(my_joint_state_publisher SHARED src/my_joint_state_publisher.cpp)
add_library(gym_training_plugin SHARED src/gym_training_plugin.cpp)
add_library(robot_contact_plugin SHARED src/robot_contact_plugin.cpp)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(parameter_server_interfaces REQUIRED)
find_package(ros2_control_interfaces REQUIRED)
find_package(py_wrappers REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(kdl_parser REQUIRED)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
target_include_directories(robot_control_plugin PUBLIC include)
ament_target_dependencies(
        robot_control_plugin
        gazebo_dev
        gazebo_ros
        rclcpp
        std_msgs
        std_srvs
        parameter_server_interfaces
        ros2_control_interfaces
        py_wrappers
        ament_index_cpp
        # The following 3 dependencies are because of the forward kinematics library
        # Need to figure out some way such that it can link automatically to those rather than linking again
        urdf
        urdfdom_headers
        kdl_parser
)
target_include_directories(my_joint_state_publisher PUBLIC include)
ament_target_dependencies(
        my_joint_state_publisher
        gazebo_dev
        gazebo_ros
        rclcpp
        sensor_msgs
        parameter_server_interfaces
)
target_include_directories(gym_training_plugin PUBLIC include)
ament_target_dependencies(
        gym_training_plugin
        gazebo_dev
        gazebo_ros
        rclcpp
        parameter_server_interfaces
        ros2_control_interfaces
)
target_include_directories(robot_contact_plugin PUBLIC include)
ament_target_dependencies(
        robot_contact_plugin
        gazebo_dev
        gazebo_ros
        rclcpp
        parameter_server_interfaces
        ros2_control_interfaces
)


install(
        TARGETS robot_control_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(
        TARGETS my_joint_state_publisher
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(
        TARGETS gym_training_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(
        TARGETS robot_contact_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

#install(TARGETS robot_plugin
#  DESTINATION lib/${PROJECT_NAME}/
#)
#install(TARGETS my_joint_state_publisher
#DESTINATION lib/${PROJECT_NAME}/
#)
install(DIRECTORY include/
        DESTINATION include)

ament_export_include_directories(
        include
)
ament_export_libraries(
        robot_control_plugin
        my_joint_state_publisher
        gym_training_plugin
        robot_contact_plugin
)

ament_package()
