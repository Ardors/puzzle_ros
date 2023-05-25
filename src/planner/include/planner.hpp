#ifndef PLANNER_HPP_
#define PLANNER_HPP_

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
#include "interfaces/srv/locate_pieces.hpp"
#include "interfaces/action/solve_puzzle.hpp"
#include "ur_msgs/srv/set_io.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"


using SolvePuzzle = interfaces::action::SolvePuzzle;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class Planner : public rclcpp::Node {
 public:
  explicit Planner(const std::string& name);
  ~Planner();

  // Init node
  bool init();

  std::shared_ptr<rclcpp_action::Client<FollowJointTrajectory>> action_joint_;

  std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>>
      publisher_joints_;

  // IdentifyPiece client and its callback
  rclcpp::CallbackGroup::SharedPtr callback_group_client_identify_piece_;
  std::shared_ptr<rclcpp::Client<interfaces::srv::IdentifyPiece>> client_identify_piece_;

  // LocatePieces client and its callback
  rclcpp::CallbackGroup::SharedPtr callback_group_client_locate_pieces_;
  std::shared_ptr<rclcpp::Client<interfaces::srv::LocatePieces>> client_locate_pieces_;

  // SetIO client and its callback
  rclcpp::CallbackGroup::SharedPtr callback_group_client_set_io_;
  std::shared_ptr<rclcpp::Client<ur_msgs::srv::SetIO>> client_set_io_;

  // SolvePuzzle action and its callback
  rclcpp::CallbackGroup::SharedPtr callback_group_action_solve_puzzle_;
  rclcpp_action::Server<SolvePuzzle>::SharedPtr action_solve_puzzle_;

 private:
  bool requestLocatePieces();
  bool requestIdentifyPiece();
  bool requestSetIO(uint8_t pin, uint8_t state);
  bool operateGripper(bool open);
  bool moveRobot(float angles[6]);

  // Function related to SolvePuzzle action
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SolvePuzzle::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SolvePuzzle>> goal_handle);
  void solvePuzzleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SolvePuzzle>> goal_handle); 
  void executeSolvePuzzle(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SolvePuzzle>> goal_handle);

};

  // Definir posiciones
  float posicion_test_a[] = {-0.9, -1.2, -0.8, 5, 3.4, -2.8};
  float posicion_test_b[] = {0, 0, 0.8, 5, 3.4, -2.8};

#endif  // PLANNER_HPP_