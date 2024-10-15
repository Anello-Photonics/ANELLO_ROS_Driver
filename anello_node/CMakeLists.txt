cmake_minimum_required(VERSION 3.8)
project(anello_ros_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/APIMU.msg"
  "msg/APIM1.msg"
  "msg/APINS.msg"
  "msg/APGPS.msg"
  "msg/APHDG.msg"
  "msg/APODO.msg"
  "msg/APHEALTH.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(anello_ros_driver_node 
  src/anello_ros_driver/main_anello_ros_driver.cpp 
  src/anello_ros_driver/bit_tools.cpp
  src/anello_ros_driver/comm/serial_interface.cpp
  src/anello_ros_driver/comm/ethernet_interface.cpp
  src/anello_ros_driver/comm/anello_config_port.cpp
  src/anello_ros_driver/comm/anello_data_port.cpp

  src/anello_ros_driver/messaging/ascii_decoder.cpp
  src/anello_ros_driver/messaging/health_message.cpp
  src/anello_ros_driver/messaging/message_publisher.cpp
  src/anello_ros_driver/messaging/ntrip_buffer.cpp
  src/anello_ros_driver/messaging/rtcm_decoder.cpp
)
ament_target_dependencies(anello_ros_driver_node 
  rclcpp 
  std_msgs 
  nmea_msgs 
  mavros_msgs
)

target_include_directories(anello_ros_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)

# Link the executable to the required libraries.
rosidl_target_interfaces(anello_ros_driver_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Install nodes
install(TARGETS
  anello_ros_driver_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)


ament_package()
