cmake_minimum_required(VERSION 3.5)
project(lobot_gazebo)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(robot_plugin SHARED src/robot_plugin.cpp)
add_library(my_joint_state_publisher SHARED src/my_joint_state_publisher.cpp)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(parameter_server_interfaces REQUIRED)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
target_include_directories(robot_plugin PUBLIC include)
ament_target_dependencies(
  robot_plugin
  gazebo_dev
  gazebo_ros
  rclcpp
  std_msgs
  parameter_server_interfaces
)
target_include_directories(my_joint_state_publisher PUBLIC include)
ament_target_dependencies(
  robot_plugin
  gazebo_dev
  gazebo_ros
  rclcpp
  std_msgs
  parameter_server_interfaces
)
ament_target_dependencies(
  my_joint_state_publisher
  gazebo_dev
  gazebo_ros
  rclcpp
  sensor_msgs
  parameter_server_interfaces
)
install(
  TARGETS robot_plugin
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

install(TARGETS robot_plugin
  DESTINATION lib/${PROJECT_NAME}/
)
install(TARGETS my_joint_state_publisher
DESTINATION lib/${PROJECT_NAME}/
)
install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(
  include
)
ament_export_libraries(
  robot_plugin
  my_joint_state_publisher
)

ament_package()
