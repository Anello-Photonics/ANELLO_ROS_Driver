cmake_minimum_required(VERSION 3.8)
project(anello_ros_driver)

# Select the build system from the ROS environment sourced by the user.
if("$ENV{ROS_VERSION}" STREQUAL "2")
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/GPSControl.srv"
    "msg/GroundGen2VBW.msg"
    "msg/GroundGen2GGA.msg"
    "msg/GroundGen2RMC.msg"
    "msg/GroundGen2HDT.msg"
    "msg/GroundGen2XDR.msg"
    "msg/GroundGen2ZDA.msg"
    "msg/GroundGen2PAPIMU.msg"
    "msg/GroundGen2PAPINS.msg"
    "msg/GroundGen2PAPACC.msg"
    DEPENDENCIES std_msgs)
  # rosidl installs the generated anello_ros_driver Python package. Install
  # the separate, shared ground_gen2 module without duplicating that target.
  ament_get_python_install_dir(PYTHON_INSTALL_DIR)
  install(DIRECTORY src/ground_gen2 DESTINATION ${PYTHON_INSTALL_DIR}
    FILES_MATCHING PATTERN "*.py")
  install(PROGRAMS scripts/ground_gen2_ros2.py DESTINATION lib/${PROJECT_NAME})
  install(FILES launch/ground_gen2.launch.py DESTINATION share/${PROJECT_NAME}/launch)
  install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
  if(BUILD_TESTING)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    enable_testing()
    add_test(NAME ground_gen2_tests COMMAND ${Python3_EXECUTABLE}
      -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/tests -p test_ground_gen2*.py)
    set_tests_properties(ground_gen2_tests PROPERTIES
      ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src")
  endif()
  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
  return()
endif()

# ROS 1 (Python 3 / Noetic).
find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation)
catkin_python_setup()
add_message_files(FILES
  GroundGen2VBW.msg
  GroundGen2GGA.msg
  GroundGen2RMC.msg
  GroundGen2HDT.msg
  GroundGen2XDR.msg
  GroundGen2ZDA.msg
  GroundGen2PAPIMU.msg
  GroundGen2PAPINS.msg
  GroundGen2PAPACC.msg)
add_service_files(FILES GPSControl.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS rospy std_msgs message_runtime)
catkin_install_python(PROGRAMS scripts/ground_gen2_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES launch/ground_gen2.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
if(CATKIN_ENABLE_TESTING)
  find_package(PythonInterp 3 REQUIRED)
  add_test(NAME ground_gen2_tests COMMAND ${PYTHON_EXECUTABLE}
    -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/tests -p test_ground_gen2*.py)
  set_tests_properties(ground_gen2_tests PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src")
endif()
