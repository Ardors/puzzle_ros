#include "gui.hpp"

using namespace std::chrono_literals;

Gui::Gui(const std::string &name) : Node(name, "") {
    if (name.empty()) {
        throw std::invalid_argument("Empty node name");
    }

}

Gui::~Gui() {}

  bool Gui::init()
  {

    // CallbackGroup to run PublishImage subscriber in a separated thread
    callback_group_subscriber_publish_image_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive); 
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_subscriber_publish_image_;

    subscriber_publish_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        "vision/publish_image",
        100, std::bind(&Gui::printImage,
        this, std::placeholders::_1), sub_opt);
    RCLCPP_INFO(get_logger(), "Subscribed to topic vision/publish_image");

    // Create LoadPuzzle client callback
    callback_group_client_load_puzzle_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    client_load_puzzle_ = this->create_client<interfaces::srv::LoadPuzzle>("vision/load_puzzle",
        rmw_qos_profile_services_default, callback_group_client_load_puzzle_);

    RCLCPP_INFO(get_logger(), "Node initialized.");

    requestLoadPuzzle();

    return true;
  }

  void Gui::printImage(const sensor_msgs::msg::Image::SharedPtr image){
    RCLCPP_INFO(get_logger(), "Image:\n%d %d %d %d %d %d\n%d %d %d %d %d %d",
    image->data[0], image->data[1], image->data[2], image->data[3], image->data[4], image->data[5],
    image->data[6], image->data[7], image->data[8], image->data[9], image->data[10], image->data[11]);
  }

  bool Gui::requestLoadPuzzle()
  {
    if (!client_load_puzzle_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(get_logger(), "vision/load_puzzle not available after waiting");
      return false;
    }

    auto request = std::make_shared<interfaces::srv::LoadPuzzle::Request>();

    // Todo esto es un test, donde cargo una imagen inventada
    request->m = 2;
    request->n = 3;

    sensor_msgs::msg::Image image;
    image.height = 1;
    image.width = 1;
    image.encoding = "brg8";
    image.is_bigendian = false;
    image.step = 3*2;
    image.data.push_back(0);
    image.data.push_back(255);
    image.data.push_back(0);

    request->face0 = image;
    request->face1 = image;
    request->face2 = image;
    request->face3 = image;
    request->face4 = image;
    request->face5 = image;

    auto future_result = client_load_puzzle_->async_send_request(request);

    if (future_result.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "vision/load_puzzle no response");
      return false;
    } else {
      auto response_value = future_result.get();
      RCLCPP_INFO(get_logger(), "LoadPuzzle service success");
      return true;
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

  rclcpp::executors::MultiThreadedExecutor exe;

  std::shared_ptr<Gui> node = std::make_shared<Gui>("gui");

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

