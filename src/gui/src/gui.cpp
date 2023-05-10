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

    RCLCPP_INFO(get_logger(), "Node initialized.");

    return true;
  }

  void Gui::printImage(const sensor_msgs::msg::Image::SharedPtr image){
    RCLCPP_INFO(get_logger(), "Image:\n%d %d %d %d %d %d\n%d %d %d %d %d %d",
    image->data[0], image->data[1], image->data[2], image->data[3], image->data[4], image->data[5],
    image->data[6], image->data[7], image->data[8], image->data[9], image->data[10], image->data[11]);
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

