cmake_minimum_required(VERSION 3.8)
project(rc_car_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Maneuver.action"
  DEPENDENCIES action_msgs
)

ament_package()
