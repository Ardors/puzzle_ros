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
#include "interfaces/srv/load_puzzle.hpp"


class Vision : public rclcpp::Node {
 public:
  explicit Vision(const std::string& name);
  ~Vision();

  // Init node
  bool init();

  // Publishers and its corresponding timers
  std::shared_ptr<rclcpp::TimerBase> timer_publish_image_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> publisher_publish_image_;

  // Services and its corresponding callbackGroups
  rclcpp::CallbackGroup::SharedPtr callback_group_service_identify_piece_;
  std::shared_ptr<rclcpp::Service<interfaces::srv::IdentifyPiece>> service_identify_piece_;

  rclcpp::CallbackGroup::SharedPtr callback_group_service_locate_pieces_;
  std::shared_ptr<rclcpp::Service<interfaces::srv::LocatePieces>> service_locate_pieces_;

  rclcpp::CallbackGroup::SharedPtr callback_group_service_load_puzzle_;
  std::shared_ptr<rclcpp::Service<interfaces::srv::LoadPuzzle>> service_load_puzzle_;

 private:

  void publishImage();

  void identifyPieceServiceCallback(const std::shared_ptr<interfaces::srv::IdentifyPiece::Request> request,
      std::shared_ptr<interfaces::srv::IdentifyPiece::Response> response);

  void locatePiecesServiceCallback(const std::shared_ptr<interfaces::srv::LocatePieces::Request> request,
      std::shared_ptr<interfaces::srv::LocatePieces::Response> response);

  void loadPuzzleServiceCallback(const std::shared_ptr<interfaces::srv::LoadPuzzle::Request> request,
      std::shared_ptr<interfaces::srv::LoadPuzzle::Response> response);
};

#endif  // VISION_HPP_