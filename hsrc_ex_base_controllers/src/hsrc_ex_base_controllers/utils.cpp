/// @brief 便利関数
/// @Copyright (C) 2021 Toyota Motor Corporation

#include "utils.hpp"

namespace hsrc_ex_base_controllers {

// 非正の場合，デフォルト値を使うパラメータ取得
double GetPositiveParameter(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& parameter_name, double default_value) {
  auto value = GetParameter(node, parameter_name, default_value);
  if (value > 0.0) {
    return value;
  } else {
    RCLCPP_WARN_STREAM(node->get_logger(),
                       parameter_name << " must be positive. Use default value " << default_value);
    return default_value;
  }
}

}  // namespace hsrc_ex_base_controllers
