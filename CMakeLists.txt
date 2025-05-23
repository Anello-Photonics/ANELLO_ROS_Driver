cmake_minimum_required(VERSION 3.0.2)
project(anello_ros_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # geometry_msgs
  # nav_msgs
  roscpp
  rospy
  # sensor_msgs
  std_msgs
  std_srvs
  message_generation
  nmea_msgs
  mavros_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  APIMU.msg
  APIM1.msg
  APINS.msg
  APGPS.msg
  APHDG.msg
  APODO.msg
  APHEALTH.msg
  APAHRS.msg
  APCOV.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  set_heading_no_unc.srv
  set_heading_with_unc.srv
  cmd_and_rsp.srv
#   Service1.srv
#   Service2.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs message_runtime nmea_msgs mavros_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/anello_ros_driver
  ${catkin_INCLUDE_DIRS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/anello_ros_driver_node.cpp)
#add_executable(anello_ros_driver 
#  src/main_anello_ros_driver.cpp 
#  src/bit_tools.cpp 
#  
#  src/messaging/rtcm_decoder.cpp 
#  src/messaging/ascii_decoder.cpp 
#  src/messaging/message_publisher.cpp 
#  src/messaging/message_subscriber.cpp
#  src/messaging/ntrip_buffer.cpp
#  src/messaging/health_message.cpp
#  
#  src/comm/serial_interface.cpp
#)

add_executable(anello_ros_driver 
  src/anello_ros_driver/main_anello_ros_driver.cpp 
  src/anello_ros_driver/bit_tools.cpp 
  
  src/anello_ros_driver/messaging/rtcm_decoder.cpp 
  src/anello_ros_driver/messaging/ascii_decoder.cpp 
  src/anello_ros_driver/messaging/message_publisher.cpp 
  src/anello_ros_driver/messaging/message_subscriber.cpp
  src/anello_ros_driver/messaging/ntrip_buffer.cpp
  src/anello_ros_driver/messaging/health_message.cpp
  
  src/anello_ros_driver/comm/serial_interface.cpp
  src/anello_ros_driver/comm/ethernet_interface.cpp
  src/anello_ros_driver/comm/anello_config_port.cpp
  src/anello_ros_driver/comm/anello_data_port.cpp
)

add_dependencies(anello_ros_driver ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
#  in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/ntrip_client/ntrip_ros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_anello_ros_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
