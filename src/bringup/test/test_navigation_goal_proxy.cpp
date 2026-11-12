#include <chrono>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "../src/navigation_goal_proxy_node.cpp"

namespace
{
using Action = nav2_msgs::action::NavigateToPose;

class NavigationGoalProxyIntegration : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    test_node_ = std::make_shared<rclcpp::Node>("navigation_goal_proxy_integration_test");
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("public_action", "/test/navigate_to_pose"),
      rclcpp::Parameter("internal_action", "/test/navigate_to_pose_internal"),
      rclcpp::Parameter("navigation_ready_topic", "/test/navigation_ready"),
      rclcpp::Parameter("goal_pose_topic", ""),
      rclcpp::Parameter("manage_goal_routes", false),
      rclcpp::Parameter("global_frame", "map")});
    proxy_ = std::make_shared<bringup::NavigationGoalProxy>(options);
    client_ = rclcpp_action::create_client<Action>(test_node_, "/test/navigate_to_pose");
    ready_pub_ = test_node_->create_publisher<std_msgs::msg::Bool>(
      "/test/navigation_ready", rclcpp::QoS(1).reliable().transient_local());
    status_sub_ = test_node_->create_subscription<std_msgs::msg::String>(
      "/navigation/goal_manager_status", rclcpp::QoS(10).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr status) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        statuses_.push_back(status->data);
      });
    executor_.add_node(test_node_);
    executor_.add_node(proxy_);
    spin_thread_ = std::thread([this]() {executor_.spin();});
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.remove_node(proxy_);
    executor_.remove_node(test_node_);
    proxy_.reset();
    test_node_.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
  }

  bool waitForStatus(const std::string & text, const std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        for (const auto & status : statuses_) {
          if (status.find(text) != std::string::npos) return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  Action::Goal makeGoal(const double x)
  {
    Action::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = static_cast<builtin_interfaces::msg::Time>(test_node_->now());
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = 1.0;
    goal.pose.pose.orientation.w = 1.0;
    return goal;
  }

  bool waitForServer()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
      if (client_->action_server_is_ready()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<bringup::NavigationGoalProxy> proxy_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spin_thread_;
  std::mutex status_mutex_;
  std::vector<std::string> statuses_;
};

TEST_F(NavigationGoalProxyIntegration, ReplacementAbortsOldPublicTaskAndCancelEndsCurrentTask)
{
  ASSERT_TRUE(waitForServer());

  auto first_future = client_->async_send_goal(makeGoal(1.0));
  ASSERT_EQ(first_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto first_goal = first_future.get();
  ASSERT_NE(first_goal, nullptr);
  ASSERT_TRUE(waitForStatus("已接收最终目标", std::chrono::seconds(2)));
  auto first_result_future = client_->async_get_result(first_goal);
  EXPECT_EQ(first_result_future.wait_for(std::chrono::milliseconds(100)),
    std::future_status::timeout);

  auto replacement_future = client_->async_send_goal(makeGoal(3.0));
  ASSERT_EQ(replacement_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto replacement_goal = replacement_future.get();
  ASSERT_NE(replacement_goal, nullptr);

  ASSERT_EQ(first_result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_EQ(first_result_future.get().code, rclcpp_action::ResultCode::ABORTED);

  std_msgs::msg::Bool ready;
  ready.data = true;
  ready_pub_->publish(ready);
  auto cancel_future = client_->async_cancel_goal(replacement_goal);
  ASSERT_EQ(cancel_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_FALSE(cancel_future.get()->goals_canceling.empty());
  auto replacement_result_future = client_->async_get_result(replacement_goal);
  ASSERT_EQ(replacement_result_future.wait_for(std::chrono::seconds(3)),
    std::future_status::ready);
  EXPECT_EQ(replacement_result_future.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_TRUE(waitForStatus("已取消", std::chrono::seconds(2)));
}
}  // namespace
