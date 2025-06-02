cmake_minimum_required(VERSION 3.5)
project(hsrb_gripper_controller)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tmc_control_msgs REQUIRED)
find_package(tmc_exxx_servo_motor_protocol REQUIRED)
find_package(urdf REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/hrh_gripper_controller.cpp
  src/hrh_gripper_follow_trajectory_action.cpp
  src/hrh_gripper_follow_distance_trajectory_action.cpp
  src/hrh_gripper_grasp_action.cpp
  src/hrh_gripper_apply_force_action.cpp
  src/hrh_gripper_set_distance_action.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  joint_trajectory_controller
  pluginlib
  rclcpp
  rclcpp_action
  tmc_control_msgs
  tmc_exxx_servo_motor_protocol
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

  ament_add_gtest(set_distance_action_test test/hrh_gripper_set_distance_action_test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(set_distance_action_test ${PROJECT_NAME})

  ament_add_gtest(set_distance_calculator_test test/hrh_gripper_set_distance_calculator_test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(set_distance_calculator_test ${PROJECT_NAME})

  ament_add_gtest(follow_distance_trajectory_action_test test/hrh_gripper_follow_distance_trajectory_action_test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(follow_distance_trajectory_action_test ${PROJECT_NAME})
endif()


ament_export_dependencies(
  controller_interface
  joint_trajectory_controller
  rclcpp
  rclcpp_action
  tmc_control_msgs
  tmc_exxx_servo_motor_protocol
  urdf
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
