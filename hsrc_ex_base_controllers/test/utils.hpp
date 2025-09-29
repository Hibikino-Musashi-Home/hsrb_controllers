/// @brief テスト用の便利関数
/// @copyright Copyright (C) 2022 Toyota Motor Corporation

#include <fstream>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

namespace hsrc_ex_base_controllers {

template<typename TYPE>
class SubscriptionCounter {
 public:
  using Ptr = std::shared_ptr<SubscriptionCounter>;

  SubscriptionCounter(const rclcpp::Node::SharedPtr& node, const std::string& topic_name) : count_(0) {
    subscriber_ = node->template create_subscription<TYPE>(
        topic_name, 1, std::bind(&SubscriptionCounter<TYPE>::Callback, this, std::placeholders::_1));
  }

  uint32_t count() const { return count_; }
  TYPE last_msg() const { return last_msg_; }

 private:
  void Callback(const typename TYPE::SharedPtr msg) {
    ++count_;
    last_msg_ = *msg;
  }
  typename rclcpp::Subscription<TYPE>::SharedPtr subscriber_;
  uint32_t count_;
  TYPE last_msg_;
};

template<typename TYPE>
class TopicRelay {
 public:
  using Ptr = std::shared_ptr<TopicRelay>;

  TopicRelay(const rclcpp::Node::SharedPtr& node, const std::string& in_name, const std::string& out_name) {
    sub_ = node->template create_subscription<TYPE>(
        in_name, 1, std::bind(&TopicRelay<TYPE>::Callback, this, std::placeholders::_1));
    pub_ = node->template create_publisher<TYPE>(out_name, rclcpp::SystemDefaultsQoS());
  }

 private:
  void Callback(const typename TYPE::SharedPtr msg) { pub_->publish(*msg); }

  typename rclcpp::Subscription<TYPE>::SharedPtr sub_;
  typename rclcpp::Publisher<TYPE>::SharedPtr pub_;
};

class TimeoutDetection {
 public:
  explicit TimeoutDetection(const rclcpp::Clock::SharedPtr& clock, double timeout_sec = 1.0) {
    clock_ = clock;
    timeout_stamp_ = clock_->now() + rclcpp::Duration::from_seconds(timeout_sec);
  }

  void Run() {
    if (clock_->now() > timeout_stamp_) {
      FAIL();
    }
  }

 private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time timeout_stamp_;
};

std::string GetRobotDescription() {
  std::fstream xml_file("robot.xml", std::fstream::in);
  std::string robot_description;
  while (xml_file.good()) {
    std::string line;
    std::getline(xml_file, line);
    robot_description += (line + "\n");
  }
  xml_file.close();
  return robot_description;
}

}  // namespace hsrc_ex_base_controllers
