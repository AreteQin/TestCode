#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using namespace std::chrono_literals;

// A minimal lifecycle node that logs lifecycle state transitions.
class MinimalLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MinimalLifecycleNode(const std::string & node_name)
  : rclcpp_lifecycle::LifecycleNode(
      node_name,               // The node name.
      "",                      // Use the default namespace.
      // Node options: enable intra-process communications and remap the internal node name.
      rclcpp::NodeOptions()
        .use_intra_process_comms(true)
        .arguments({"--ros-args", "-r",
                    node_name + ":" + std::string("__node:=") + node_name}))
  {
    RCLCPP_INFO(get_logger(), "Lifecycle node '%s' constructed", node_name.c_str());
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_configure() called");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_activate() called");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_deactivate() called");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_cleanup() called");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_shutdown() called");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string node_name = "my_lifecycle_node";
  auto lifecycle_node = std::make_shared<MinimalLifecycleNode>(node_name);

  // Create an executor to run the node.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_node->get_node_base_interface());

  // Trigger the CONFIGURE transition and check the returned state.
  RCLCPP_INFO(lifecycle_node->get_logger(), "Triggering CONFIGURE transition...");
  auto state = lifecycle_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(lifecycle_node->get_logger(), "Failed to configure node, current state: %d", state.id());
  } else {
    RCLCPP_INFO(lifecycle_node->get_logger(), "Node configured successfully.");
  }

  // Trigger the ACTIVATE transition and check the returned state.
  RCLCPP_INFO(lifecycle_node->get_logger(), "Triggering ACTIVATE transition...");
  state = lifecycle_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(lifecycle_node->get_logger(), "Failed to activate node, current state: %d", state.id());
  } else {
    RCLCPP_INFO(lifecycle_node->get_logger(), "Node activated successfully.");
  }

  // Let the node run for a few seconds.
  RCLCPP_INFO(lifecycle_node->get_logger(), "Node is now active and running...");
  for (int i = 0; i < 5; ++i) {
    executor.spin_once(100ms);
  }

  // Trigger the DEACTIVATE transition.
  RCLCPP_INFO(lifecycle_node->get_logger(), "Triggering DEACTIVATE transition...");
  state = lifecycle_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(lifecycle_node->get_logger(), "Failed to deactivate node, current state: %d", state.id());
  } else {
    RCLCPP_INFO(lifecycle_node->get_logger(), "Node deactivated successfully.");
  }

  // Trigger the CLEANUP transition.
  RCLCPP_INFO(lifecycle_node->get_logger(), "Triggering CLEANUP transition...");
  state = lifecycle_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    RCLCPP_ERROR(lifecycle_node->get_logger(), "Failed to cleanup node, current state: %d", state.id());
  } else {
    RCLCPP_INFO(lifecycle_node->get_logger(), "Node cleaned up successfully.");
  }

  rclcpp::shutdown();
  return 0;
}
