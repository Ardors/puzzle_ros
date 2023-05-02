#ifndef VISION_HPP_
#define VISION_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "interfaces/srv/identify_piece.hpp"
#include "interfaces/srv/locate_pieces.hpp"

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

  // Publishers and its corresponding timers
  std::shared_ptr<rclcpp::TimerBase> timer_publish_image_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> publisher_publish_image_;

  // Services and its corresponding callbackGroups
  rclcpp::CallbackGroup::SharedPtr callback_group_service_identify_piece_;
  std::shared_ptr<rclcpp::Service<interfaces::srv::IdentifyPiece>> service_identify_piece_;

  rclcpp::CallbackGroup::SharedPtr callback_group_service_locate_pieces_;
  std::shared_ptr<rclcpp::Service<interfaces::srv::LocatePieces>> service_locate_pieces_;

 private:

  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

  void publishImage();

  void identifyPieceServiceCallback(const std::shared_ptr<interfaces::srv::IdentifyPiece::Request> request,
      std::shared_ptr<interfaces::srv::IdentifyPiece::Response> response);

  void locatePiecesServiceCallback(const std::shared_ptr<interfaces::srv::LocatePieces::Request> request,
      std::shared_ptr<interfaces::srv::LocatePieces::Response> response);
};

#endif  // VISION_HPP_