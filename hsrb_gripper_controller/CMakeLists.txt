cmake_minimum_required(VERSION 3.5)
project(hsrb_gripper_controller)

add_compile_options(-Wall -Wno-nonnull)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tmc_control_msgs REQUIRED)
find_package(tmc_exxx_servo_motor_protocol REQUIRED)
find_package(tmc_utils REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/hrh_gripper_controller.cpp
  src/hrh_gripper_distance.cpp
  src/hrh_gripper_follow_trajectory_action.cpp
  src/hrh_gripper_follow_distance_trajectory_action.cpp
  src/hrh_gripper_grasp_action.cpp
  src/hrh_gripper_apply_force_action.cpp
  src/hrh_gripper_set_distance_action.cpp
  src/hrh_gripper_state.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  control_msgs
  controller_interface
  hardware_interface
  joint_trajectory_controller
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  tmc_control_msgs
  tmc_exxx_servo_motor_protocol
  tmc_utils
  trajectory_msgs
  urdf
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

pluginlib_export_plugin_description_file(controller_interface controllers_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(apply_force_action_test test/hrh_gripper_apply_force_action_test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(apply_force_action_test ${PROJECT_NAME})

  ament_add_gtest(apply_force_calculator_test test/hrh_gripper_controller_apply_force_calculator_test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(apply_force_calculator_test ${PROJECT_NAME})

  ament_add_gtest(gripper_controller_test test/hrh_gripper_controller_test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(gripper_controller_test ${PROJECT_NAME})

  ament_add_gtest(follow_trajectory_action_test test/hrh_gripper_follow_trajectory_action_test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(follow_trajectory_action_test ${PROJECT_NAME})

  ament_add_gtest(grasp_action_test test/hrh_gripper_grasp_action_test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(grasp_action_test ${PROJECT_NAME})

  ament_add_gtest(hrh_gripper_controller_output_position_control_test test/hrh_gripper_controller_output_position_control_test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(hrh_gripper_controller_output_position_control_test ${PROJECT_NAME})

  ament_add_gtest(set_distance_action_test test/hrh_gripper_set_distance_action_test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(set_distance_action_test ${PROJECT_NAME})

  ament_add_gtest(distance_calculator_test test/hrh_gripper_distance_calculator_test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(distance_calculator_test ${PROJECT_NAME})

  ament_add_gtest(follow_distance_trajectory_action_test test/hrh_gripper_follow_distance_trajectory_action_test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(follow_distance_trajectory_action_test ${PROJECT_NAME})

  ament_add_gtest(gripper_state_test test/hrh_gripper_state_test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(gripper_state_test ${PROJECT_NAME})

  ament_add_gtest(gripper_distance_test test/hrh_gripper_distance_test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(gripper_distance_test ${PROJECT_NAME})
endif()


ament_export_dependencies(
  control_msgs
  controller_interface
  hardware_interface
  joint_trajectory_controller
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  tmc_control_msgs
  tmc_exxx_servo_motor_protocol
  tmc_utils
  trajectory_msgs
  urdf
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
