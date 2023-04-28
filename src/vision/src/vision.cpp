#include "vision.hpp"

using namespace std::chrono_literals;

Vision::Vision(const std::string &name) : LifecycleNode(name, "") {
    if (name.empty()) {
        throw std::invalid_argument("Empty node name");
    }

}

Vision::~Vision() {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Vision::on_configure(const rclcpp_lifecycle::State &)
  {
    // Timer to run PublishImage publisher
    publisher_publish_image_ = this->create_publisher<sensor_msgs::msg::Image>("vision/publish_image", 10);
    timer_publish_image_ = this->create_wall_timer(
      1s, std::bind(&Vision::publishImage, this));
      
    // CallbackGroup to run IdentifyPiece service in a separated thread
    callback_group_service_identify_piece_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    service_identify_piece_ = this->create_service<interfaces::srv::IdentifyPiece>("vision/identify_piece",
        std::bind(&Vision::identifyPieceServiceCallback, this,
        std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_service_identify_piece_);
    RCLCPP_INFO(get_logger(), "Service IdentifyPiece created.");

    // CallbackGroup to run LocatePieces service in a separated thread
    callback_group_service_locate_pieces_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    service_locate_pieces_ = this->create_service<interfaces::srv::LocatePieces>("vision/locate_pieces",
        std::bind(&Vision::locatePiecesServiceCallback, this,
        std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_service_locate_pieces_);
    RCLCPP_INFO(get_logger(), "Service LocatePieces created.");

    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Vision::on_activate(const rclcpp_lifecycle::State &)
  {
    publisher_publish_image_->on_activate();

    RCLCPP_INFO(get_logger(), "on_activate() is called.");

    std::this_thread::sleep_for(2s);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Vision::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_deactivate() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Vision::on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on cleanup is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Vision::on_error(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on error is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Vision::on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "on shutdown is called from state %s.", state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void Vision::identifyPieceServiceCallback(const std::shared_ptr<interfaces::srv::IdentifyPiece::Request> request,
      std::shared_ptr<interfaces::srv::IdentifyPiece::Response> response){
      
      RCLCPP_INFO(get_logger(), "IdentifyPiece service called");

      response->piece.piece_id = 2;
      response->piece.piece_orientation = 1;
  }

  void Vision::locatePiecesServiceCallback(const std::shared_ptr<interfaces::srv::LocatePieces::Request> request,
      std::shared_ptr<interfaces::srv::LocatePieces::Response> response){
      
      RCLCPP_INFO(get_logger(), "LocatePieces service called");

      interfaces::msg::PiecePose temp;

      response->size = 2;

      temp.piece_x = 0.02;
      temp.piece_y = 0.3;
      temp.piece_w = 1.2;
      response->poses.push_back(temp);

      temp.piece_x = 0.04;
      temp.piece_y = 0.5;
      temp.piece_w = 1.3;
      response->poses.push_back(temp);
  }

  void Vision::publishImage()
  {
    sensor_msgs::msg::Image image;
    char data[] = {0,0,0,0,0,0,255,0,0,0,0,0};
    image.height = 2;
    image.width = 2;
    image.encoding = "brg8";
    image.is_bigendian = false;
    image.step = 3*2;
    image.data.push_back(0);
    image.data.push_back(255);
    image.data.push_back(0);
    image.data.push_back(0);
    image.data.push_back(0);
    image.data.push_back(0);

    image.data.push_back(0);
    image.data.push_back(0);
    image.data.push_back(0);
    image.data.push_back(0);
    image.data.push_back(255);
    image.data.push_back(0);

    publisher_publish_image_->publish(std::move(image));
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

  std::shared_ptr<Vision> lc_node =
    std::make_shared<Vision>("vision");

  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
