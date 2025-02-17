#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class MyNode : public rclcpp_lifecycle::LifecycleNode {
public:
  MyNode() : rclcpp_lifecycle::LifecycleNode("my_node") {

    // Declare a parameter with the command line argument name
    // Method 1
    this->declare_parameter("my_param", rclcpp::ParameterValue(""));
    RCLCPP_INFO(this->get_logger(), "my_param: %s", this->get_parameter("my_param").as_string().c_str());
    // // Method 2
    // // Declare a parameter with a default value.
    // this->declare_parameter<std::string>("my_param", "default_value");
    // std::string value;
    // this->get_parameter<std::string>("my_param", value);
    // RCLCPP_INFO(this->get_logger(), "my_param: %s", value.c_str());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node->get_node_base_interface());
  return 0;
}
