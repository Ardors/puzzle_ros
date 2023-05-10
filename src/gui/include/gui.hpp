#ifndef GUI_HPP_
#define GUI_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


class Gui : public rclcpp::Node {
 public:
  explicit Gui(const std::string& name);
  ~Gui();

  // Init node
  bool init();

  // Subscribers and its corresponding callbackGroups
  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_publish_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_publish_image_;

 private:
  void printImage(const sensor_msgs::msg::Image::SharedPtr image);

};

#endif  // GUI_HPP_