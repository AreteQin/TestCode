// main_task.cpp
#include <csignal>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Define a Lifecycle Node that performs work via a timer and does cleanup in its deactivation callback.
class MainTask : public rclcpp_lifecycle::LifecycleNode {
public:
  MainTask()
  : LifecycleNode("main_task")
  {
    RCLCPP_INFO(get_logger(), "MainTask constructed");
  }

  // on_configure: create resources (e.g., timer)
  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override {
    RCLCPP_INFO(get_logger(), "on_configure() called");
    timer_ = this->create_wall_timer(
      1s, std::bind(&MainTask::timer_callback, this));
    return CallbackReturn::SUCCESS;
  }

  // on_activate: node becomes fully active.
  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    RCLCPP_INFO(get_logger(), "on_activate() called");
    return CallbackReturn::SUCCESS;
  }

  // on_deactivate: perform cleanup tasks (e.g., releasing control authority).
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    RCLCPP_INFO(get_logger(), "on_deactivate() called");
    // Call cleanup function (simulate a service call)
    callReleaseAuthority();
    if (timer_) {
      timer_->cancel();
    }
    return CallbackReturn::SUCCESS;
  }

  // on_cleanup: destroy resources.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) override {
    RCLCPP_INFO(get_logger(), "on_cleanup() called");
    timer_.reset();
    return CallbackReturn::SUCCESS;
  }

  // on_shutdown: final callback on shutdown.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override {
    RCLCPP_INFO(get_logger(), "on_shutdown() called");
    return CallbackReturn::SUCCESS;
  }

  // Timer callback (dummy work)
  void timer_callback() {
    RCLCPP_INFO(get_logger(), "Timer callback executing...");
  }

  // Simulated cleanup work that might include a synchronous service call.
  void callReleaseAuthority() {
    RCLCPP_INFO(get_logger(), "Executing callReleaseAuthority() cleanup work...");
    // Simulate a delay as if waiting for a service response.
    std::this_thread::sleep_for(2s);
    RCLCPP_INFO(get_logger(), "Cleanup work complete: control authority released.");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

// Global pointers to node and executor for the custom signal handler.
std::shared_ptr<MainTask> g_node;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> g_executor;

// Custom signal handler that triggers proper lifecycle transitions before shutdown.
void customSignalHandler(int signum)
{
  RCLCPP_INFO(g_node->get_logger(), "Custom signal handler invoked: signal %d", signum);

  // Trigger deactivation (cleanup work will run inside on_deactivate)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ret;
  g_node->deactivate(ret);
  RCLCPP_INFO(g_node->get_logger(), "Deactivation complete.");

  // Optionally, trigger the shutdown transition.
  g_node->shutdown(ret);
  RCLCPP_INFO(g_node->get_logger(), "Shutdown transition complete.");

  // Finally, call rclcpp::shutdown() so the executor stops spinning.
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  // Disable automatic shutdown on signal by setting the member variable.
  rclcpp::InitOptions init_options;
  init_options.shutdown_on_signal = false;
  rclcpp::init(argc, argv, init_options);

  // Create the lifecycle node.
  g_node = std::make_shared<MainTask>();

  // Create a multi-threaded executor and add the node.
  g_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  g_executor->add_node(g_node->get_node_base_interface());

  // Register our custom signal handler for SIGINT (Ctrl+C).
  std::signal(SIGINT, customSignalHandler);

  // Trigger lifecycle transitions:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ret;
  g_node->configure(ret);
  g_node->activate(ret);

  // Spin the executor until shutdown is triggered.
  g_executor->spin();

  // Cleanup after shutdown.
  g_executor->remove_node(g_node->get_node_base_interface());
  g_node.reset();
  g_executor.reset();

  return 0;
}
