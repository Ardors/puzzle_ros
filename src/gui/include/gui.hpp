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

using LifecycleCallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Gui : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit Gui(const std::string& name);
  ~Gui();

  // Init node
  bool init();
  bool setParams(unsigned int charger_node, int connect_4g_timeout,
      const std::string& ip, const std::string& mac, int error_clear_trials,
      const int & errorsPeriod);

  // Subscribers and its corresponding callbackGroups
  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_publish_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_publish_image_;

 private:

  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

  void printImage(const sensor_msgs::msg::Image::SharedPtr image);

};

#endif  // GUI_HPP_