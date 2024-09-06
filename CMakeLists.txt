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

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(anello_ros_driver 
  src/anello_ros_driver/main_anello_ros_driver.cpp 
  src/anello_ros_driver/bit_tools.cpp

  src/anello_ros_driver/comm/serial_interface.cpp
  src/anello_ros_driver/comm/ethernet_interface.cpp
  src/anello_ros_driver/comm/anello_config_port.cpp
  src/anello_ros_driver/comm/anello_data_port.cpp
)
ament_target_dependencies(anello_ros_driver rclcpp std_msgs nmea_msgs mavros_msgs)

install(TARGETS
  anello_ros_driver
  DESTINATION lib/${PROJECT_NAME})
ament_package()

# add_executable(anello_ros_driver 
#   src/anello_ros_driver/main_anello_ros_driver.cpp 
#   src/anello_ros_driver/bit_tools.cpp 
  
#   src/anello_ros_driver/messaging/rtcm_decoder.cpp 
#   src/anello_ros_driver/messaging/ascii_decoder.cpp 
#   src/anello_ros_driver/messaging/message_publisher.cpp 
#   src/anello_ros_driver/messaging/message_subscriber.cpp
#   src/anello_ros_driver/messaging/ntrip_buffer.cpp
#   src/anello_ros_driver/messaging/health_message.cpp
  
#   src/anello_ros_driver/comm/serial_interface.cpp
#   src/anello_ros_driver/comm/ethernet_interface.cpp
#   src/anello_ros_driver/comm/anello_config_port.cpp
#   src/anello_ros_driver/comm/anello_data_port.cpp
# )

# add_dependencies(anello_ros_driver ${PROJECT_NAME}_generate_messages_cpp)
