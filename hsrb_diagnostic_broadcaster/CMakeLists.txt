cmake_minimum_required(VERSION 3.5)
project(hsrb_diagnostic_broadcaster)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(hsrb_servomotor_protocol REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(${PROJECT_NAME}
  SHARED
  src/servo_diagnostic_broadcaster.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  diagnostic_msgs
  hardware_interface
  hsrb_servomotor_protocol
  pluginlib
  realtime_tools
)

pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(servo_diagnostic_broadcaster_test test/servo_diagnostic_broadcaster-test.cpp)
  target_link_libraries(servo_diagnostic_broadcaster_test ${PROJECT_NAME})
endif()

ament_export_dependencies(
  controller_interface
  diagnostic_msgs
  hardware_interface
  hsrb_servomotor_protocol
  realtime_tools
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
