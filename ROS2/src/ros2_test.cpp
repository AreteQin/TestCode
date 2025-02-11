#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("my_node"); // Create your node

    // Create a callback group
    rclcpp::CallbackGroup::SharedPtr callback_group = rclcpp::create_callback_group();

    // Add a subscription (or timer, service, etc.) to the callback group
    auto subscription = node->create_subscription<std_msgs::msg::String>(
      "topic_name", 10,
      [&](std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(node->get_logger(), "I heard: '%s'", msg->data.c_str());
      },
      rclcpp::SubscriptionOptions(rclcpp::QoS(10), callback_group) // Associate with the group
    );

    // Create the executor
    rclcpp::executors::SingleThreadedExecutor executor;

    // Add the node to the executor (not the callback group)
    executor.add_node(node);

    // Spin the executor
    executor.spin();

    rclcpp::shutdown();
    return 0;
}