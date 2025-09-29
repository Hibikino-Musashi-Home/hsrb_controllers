/// @brief 便利関数
/// @Copyright (C) 2021 Toyota Motor Corporation
#ifndef HSRC_EX_BASE_CONTROLLERS_UTILS_HPP_
#define HSRC_EX_BASE_CONTROLLERS_UTILS_HPP_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace hsrc_ex_base_controllers {

// デフォルト値ありのパラメータ取得
template <typename ParameterType>
auto GetParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                  const std::string& name,
                  const ParameterType& default_value) {
  if (!node->has_parameter(name)) {
    return node->declare_parameter<ParameterType>(name, default_value);
  } else {
    return node->get_parameter(name).get_value<ParameterType>();
  }
}

// 非正の場合，デフォルト値を使うパラメータ取得
double GetPositiveParameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& parameter_name,
                            double default_value);

}  // namespace hsrc_ex_base_controllers

#endif  // HSRC_EX_BASE_CONTROLLERS_UTILS_HPP_
