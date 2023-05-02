#include "gui.hpp"

using namespace std::chrono_literals;

Gui::Gui(const std::string &name) : LifecycleNode(name, "") {
    if (name.empty()) {
        throw std::invalid_argument("Empty node name");
    }

}

Gui::~Gui() {}


  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Gui::on_configure(const rclcpp_lifecycle::State &)
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

    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Gui::on_activate(const rclcpp_lifecycle::State &)
  {

    RCLCPP_INFO(get_logger(), "on_activate() is called.");

    // Let's sleep for 2 seconds.
    // We emulate we are doing important
    // work in the activating phase.
    std::this_thread::sleep_for(2s);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Gui::on_deactivate(const rclcpp_lifecycle::State &)
  {

    RCLCPP_INFO(get_logger(), "on_deactivate() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Gui::on_cleanup(const rclcpp_lifecycle::State &)
  {

    RCLCPP_INFO(get_logger(), "on cleanup is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Gui::on_error(const rclcpp_lifecycle::State &)
  {

    RCLCPP_INFO(get_logger(), "on error is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Gui::on_shutdown(const rclcpp_lifecycle::State & state)
  {

    RCUTILS_LOG_INFO_NAMED(
      get_name(),
      "on shutdown is called from state %s.",
      state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void Gui::printImage(const sensor_msgs::msg::Image::SharedPtr image){
    RCLCPP_INFO(get_logger(), "%d %d %d %d %d %d\n%d %d %d %d %d %d",
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

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<Gui> lc_node =
    std::make_shared<Gui>("gui");

  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}

