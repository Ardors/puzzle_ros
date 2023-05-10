#include "planner.hpp"

using namespace std::chrono_literals;

using SolvePuzzle = interfaces::action::SolvePuzzle;

Planner::Planner(const std::string &name) : LifecycleNode(name, "") {
    if (name.empty()) {
        throw std::invalid_argument("Empty node name");
    }

}

Planner::~Planner() {}

  bool Planner::init()
  {
    // Create IdentifyVision client callback
    callback_group_client_identify_piece_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    client_identify_piece_ = this->create_client<interfaces::srv::IdentifyPiece>("vision/identify_piece",
        rmw_qos_profile_services_default, callback_group_client_identify_piece_);

    // Create LocatePieces client callback
    callback_group_client_locate_pieces_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    client_locate_pieces_ = this->create_client<interfaces::srv::LocatePieces>("vision/locate_pieces",
        rmw_qos_profile_services_default, callback_group_client_locate_pieces_);

    // Create SetIO client callback
    callback_group_client_set_io_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    client_set_io_ = this->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io",
        rmw_qos_profile_services_default, callback_group_client_set_io_);

    // Create SolvePuzzle action callback
    callback_group_action_solve_puzzle_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    const rcl_action_server_options_t & options = rcl_action_server_get_default_options();

    action_solve_puzzle_ = rclcpp_action::create_server<SolvePuzzle>(
      this,
      "planner/solve_puzzle",
      std::bind(&Planner::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Planner::handleCancel, this, std::placeholders::_1),
      std::bind(&Planner::solvePuzzleAccepted, this, std::placeholders::_1),
      options,
      callback_group_action_solve_puzzle_);

    RCLCPP_INFO(get_logger(), "Node initialized.");

    return true;
  }

  bool Planner::requestIdentifyPiece()
  {
    if (!client_identify_piece_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(get_logger(), "vision/identify_piece not available after waiting");
      return false;
    }

    auto request = std::make_shared<interfaces::srv::IdentifyPiece::Request>();
    auto future_result = client_identify_piece_->async_send_request(request);

    if (future_result.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "vision/identify_piece no response");
      return false;
    } else {
      auto response_value = future_result.get();
      RCLCPP_INFO(get_logger(), "IdentifyPiece: %d %d", response_value->piece.piece_id,response_value->piece.piece_orientation);
      return true;
    }
  }

  bool Planner::requestLocatePieces()
  {
    if (!client_locate_pieces_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(get_logger(), "vision/identify_piece not available after waiting");
      return false;
    }

    auto request = std::make_shared<interfaces::srv::LocatePieces::Request>();
    auto future_result = client_locate_pieces_->async_send_request(request);

    if (future_result.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "vision/identify_piece no response");
      return false;
    } else {
      auto response_value = future_result.get();

      RCLCPP_INFO(get_logger(), "LocatePieces:");
      for (interfaces::msg::PiecePose i : response_value->poses) {
        RCLCPP_INFO(get_logger(), "x: %f", i.piece_x);
        RCLCPP_INFO(get_logger(), "y: %f", i.piece_y);
        RCLCPP_INFO(get_logger(), "w: %f", i.piece_w);
      }
      return true;
    }
  }

  bool Planner::requestSetIO(uint8_t pin, uint8_t state){
    if (!client_set_io_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(get_logger(), "/io_and_status_controller/set_io not available after waiting");
      return false;
    }

    auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
    request->fun = 1;
    request->pin = pin;
    request->state = state;
    auto future_result = client_set_io_->async_send_request(request);

    if (future_result.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "/io_and_status_controller/set_io no response");
      return false;
    } else {
      if (future_result.get()->success) {
        auto future_result_value = future_result.get();
        return true;
      } else {
        RCLCPP_ERROR(get_logger(),"/io_and_status_controller/set_io return error");
        return false;
      }
      return true;
    }
  }

  bool Planner::operateGripper(bool open){
    bool success = true;
    if(open){
      success &= requestSetIO(16, 0);
      success &= requestSetIO(17, 1);
    }
    else{
      success &= requestSetIO(17, 0);
      success &= requestSetIO(16, 1);
    }
    return success;
  }

  rclcpp_action::GoalResponse Planner::handleGoal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const SolvePuzzle::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "SolvePuzzle action handled");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse Planner::handleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<SolvePuzzle>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "SolvePuzzle action canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void Planner::solvePuzzleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<SolvePuzzle>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "SolvePuzzle action accepted");
    std::thread{std::bind(&Planner::executeSolvePuzzle, this, std::placeholders::_1), goal_handle}.detach();
  }

  void Planner::executeSolvePuzzle(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<SolvePuzzle>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "SolvePuzzle action executing");

    auto result = std::make_shared<SolvePuzzle::Result>();
    
    if(!requestIdentifyPiece()){
      result->success = false;
      goal_handle->succeed(result);
      return;
    }

    if(!requestLocatePieces()){
      result->success = false;
      goal_handle->succeed(result);
      return;
    }

    operateGripper(true);
    operateGripper(false);

    // Set a target Pose
    auto const target_pose = []{
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 1.0;
      msg.position.x = -0.15;
      msg.position.y = -0.3;
      msg.position.z = -0.2;
      return msg;
    }();
    move_group_interface->setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface->plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
      move_group_interface->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planing failed!");
    }

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "SolvePuzzle action completed");
    return;
  }

/**
 * A lifecycle node has the same node API
 * as a regular node. This means we can spawn a
 * node, give it a name and add it to the executor.
 */
int main(int argc, char * argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exe;

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> lc_node =
    std::make_shared<Planner>("planner");
  
  // std::shared_ptr<rclcpp::Node> aux = lc_node;

  move_group_interface = new  MoveGroupInterface(lc_node,"ur_manipulator");

  // Init node
  if (!node->init()) {
    RCLCPP_ERROR(node->get_logger(), "Node initialization failed");
    rclcpp::shutdown();
    return 1;
  }

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
