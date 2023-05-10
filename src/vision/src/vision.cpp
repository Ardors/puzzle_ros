#include "vision.hpp"

using namespace std::chrono_literals;

Vision::Vision(const std::string &name) : Node(name, "") {
    if (name.empty()) {
        throw std::invalid_argument("Empty node name");
    }

}

Vision::~Vision() {}

  bool Vision::init()
  {
    // Timer to run PublishImage publisher
    publisher_publish_image_ = this->create_publisher<sensor_msgs::msg::Image>("vision/publish_image", 10);
    timer_publish_image_ = this->create_wall_timer(
      1s, std::bind(&Vision::publishImage, this));
      
    // CallbackGroup to run IdentifyPiece service in a separated thread
    callback_group_service_identify_piece_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    service_identify_piece_ = this->create_service<interfaces::srv::IdentifyPiece>("vision/identify_piece",
        std::bind(&Vision::identifyPieceServiceCallback, this,
        std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_service_identify_piece_);
    RCLCPP_INFO(get_logger(), "Service IdentifyPiece created.");

    // CallbackGroup to run LocatePieces service in a separated thread
    callback_group_service_locate_pieces_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    service_locate_pieces_ = this->create_service<interfaces::srv::LocatePieces>("vision/locate_pieces",
        std::bind(&Vision::locatePiecesServiceCallback, this,
        std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_service_locate_pieces_);
    RCLCPP_INFO(get_logger(), "Service LocatePieces created.");

    RCLCPP_INFO(get_logger(), "Node initialized.");

    return true;
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

  rclcpp::executors::MultiThreadedExecutor exe;

  std::shared_ptr<Vision> node = std::make_shared<Vision>("vision");

  exe.add_node(node->get_node_base_interface());

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
