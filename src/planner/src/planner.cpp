#include "planner.hpp"

using namespace std::chrono_literals;

Planner::Planner(const std::string &name) : LifecycleNode(name, "") {
    if (name.empty()) {
        throw std::invalid_argument("Empty node name");
    }

}

Planner::~Planner() {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Planner::on_configure(const rclcpp_lifecycle::State &)
  {
    // Create each client with its corresponding callbackGroup
    callback_group_client_piece_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    planner_client_piece_ = this->create_client<interfaces::srv::IdentifyPiece>("vision/identify_piece",
        rmw_qos_profile_services_default, callback_group_client_piece_);

    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Planner::on_activate(const rclcpp_lifecycle::State &)
  {

    RCLCPP_INFO(get_logger(), "on_activate() is called.");

    // Let's sleep for 2 seconds.
    // We emulate we are doing important
    // work in the activating phase.
    std::this_thread::sleep_for(2s);

    requestIdentifyPiece();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Planner::on_deactivate(const rclcpp_lifecycle::State &)
  {

    RCLCPP_INFO(get_logger(), "on_deactivate() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Planner::on_cleanup(const rclcpp_lifecycle::State &)
  {

    RCLCPP_INFO(get_logger(), "on cleanup is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Planner::on_error(const rclcpp_lifecycle::State &)
  {

    RCLCPP_INFO(get_logger(), "on error is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Planner::on_shutdown(const rclcpp_lifecycle::State & state)
  {

    RCUTILS_LOG_INFO_NAMED(
      get_name(),
      "on shutdown is called from state %s.",
      state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void Planner::requestIdentifyPiece()
  {
    if (!planner_client_piece_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(get_logger(), "vision/identify_piece not available after waiting");
      return;
    }

    auto request = std::make_shared<interfaces::srv::IdentifyPiece::Request>();
    auto future_result = planner_client_piece_->async_send_request(request);

    if (future_result.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "vision/identify_piece no response");
      return;
    } else {
      auto response_value = future_result.get();
      RCLCPP_INFO(get_logger(), "IdentifyPiece: %d %d", response_value->piece.piece_id,response_value->piece.piece_orientation);
    }

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

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<Planner> lc_node =
    std::make_shared<Planner>("planner");

  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
