cmake_minimum_required(VERSION 3.5)
project(hsrb_diagnostic_broadcaster)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tmc_exxx_servo_motor_protocol REQUIRED)

add_library(${PROJECT_NAME}
  SHARED
  src/servo_diagnostic_broadcaster.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  diagnostic_msgs
  hardware_interface
  pluginlib
  realtime_tools
  tmc_exxx_servo_motor_protocol
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
  realtime_tools
  tmc_exxx_servo_motor_protocol
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
