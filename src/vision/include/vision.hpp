#ifndef VISION_HPP_
#define VISION_HPP_

#include <boost/optional.hpp>
#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "interfaces/srv/identify_piece.hpp"

using LifecycleCallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Vision : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit Vision(const std::string& name);
  ~Vision();

  // Init node
  bool init();
  bool setParams(unsigned int charger_node, int connect_4g_timeout,
      const std::string& ip, const std::string& mac, int error_clear_trials,
      const int & errorsPeriod);

  // Services and its corresponding callbackGroups
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_service_piece_;
  std::shared_ptr<rclcpp::Service<interfaces::srv::IdentifyPiece>> vision_service_piece_;

 private:

  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

  void identifyPieceServiceCallback(const std::shared_ptr<interfaces::srv::IdentifyPiece::Request> request,
      std::shared_ptr<interfaces::srv::IdentifyPiece::Response> response);

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
};

#endif  // VISION_HPP_