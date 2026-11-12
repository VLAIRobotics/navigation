#include <chrono>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <action_msgs/msg/goal_status_array.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>

namespace
{
class TestGate : public BT::SyncActionNode
{
public:
  TestGate(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("success", true, "")};
  }

  BT::NodeStatus tick() override
  {
    bool success = false;
    if (!getInput("success", success)) return BT::NodeStatus::FAILURE;
    return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class TestFailure : public BT::SyncActionNode
{
public:
  TestFailure(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override {return BT::NodeStatus::FAILURE;}
};

class TestCollisionCheck : public BT::SyncActionNode
{
public:
  TestCollisionCheck(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("check_path", false, ""),
      BT::InputPort<bool>("check_goal", false, ""),
      BT::InputPort<bool>("allow_unknown", false, ""),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<nav_msgs::msg::Path>("path")};
  }

  BT::NodeStatus tick() override
  {
    bool check_path = false;
    if (!getInput("check_path", check_path)) return BT::NodeStatus::FAILURE;
    if (!check_path) return BT::NodeStatus::SUCCESS;
    return config().blackboard->get<bool>("candidate_clear") ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class TestFollowPath : public BT::SyncActionNode
{
public:
  TestFollowPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<std::string>("controller_id")};
  }

  BT::NodeStatus tick() override {return BT::NodeStatus::SUCCESS;}
};

class TestPlan : public BT::SyncActionNode
{
public:
  TestPlan(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::OutputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<std::string>("mode", "success", ""),
      BT::InputPort<std::string>("planner_id", "GridBased", "")};
  }

  BT::NodeStatus tick() override
  {
    auto blackboard = config().blackboard;
    blackboard->set("plan_attempt_count",
      blackboard->get<std::uint64_t>("plan_attempt_count") + 1);
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("goal", goal)) return BT::NodeStatus::FAILURE;
    std::string mode;
    if (!getInput("mode", mode)) return BT::NodeStatus::FAILURE;
    if (!sent_outcome_ && mode != "success") {
      const auto target_sec = mode == "stale" ? goal.header.stamp.sec + 1 :
        goal.header.stamp.sec;
      publishOutcome(goal, target_sec, mode == "late", mode == "sequence_gap" ? 2 : 1,
        mode == "wrong_request", mode == "out_of_bounds" ? "invalid_goal" : "no_path");
      sent_outcome_ = true;
      return BT::NodeStatus::FAILURE;
    }
    nav_msgs::msg::Path path;
    path.header = goal.header;
    geometry_msgs::msg::PoseStamped endpoint = goal;
    path.poses.push_back(endpoint);
    return setOutput("path", path) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  void publishOutcome(
    const geometry_msgs::msg::PoseStamped & goal, const std::int32_t target_sec,
    const bool late, const std::uint64_t sequence, const bool wrong_request,
    const std::string & state)
  {
    const auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    const auto publisher = node->create_publisher<std_msgs::msg::String>(
      "/navigation/global_plan_status", rclcpp::QoS(1).reliable().transient_local());
    const auto event = static_cast<builtin_interfaces::msg::Time>(node->now());
    const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    const auto request_sec = wrong_request ? target_sec + 1 : target_sec;
    std::ostringstream status;
    status << std::setprecision(17)
           << "instance=test-restarted-planner;seq=" << sequence
           << ";request_id=plan:" << request_sec << ':' << goal.header.stamp.nanosec << ';'
           << "state=" << state << ";frame="
           << goal.header.frame_id << ";target_sec=" << target_sec
           << ";target_nanosec=" << goal.header.stamp.nanosec
           << ";x=" << goal.pose.position.x << ";y=" << goal.pose.position.y
           << ";z=" << goal.pose.position.z
           << ";qx=" << goal.pose.orientation.x << ";qy=" << goal.pose.orientation.y
           << ";qz=" << goal.pose.orientation.z << ";qw=" << goal.pose.orientation.w
           << ";event_sec=" << event.sec << ";event_nanosec=" << event.nanosec
           << ";event_steady_ns=" << (late ? 1 : steady_ns);
    std_msgs::msg::String message;
    message.data = status.str();
    publisher->publish(message);
    outcome_publisher_ = publisher;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr outcome_publisher_;
  bool sent_outcome_{false};
};

const char * kSuccessTree = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <GlobalPlanRetry goal="{goal}" planning_goal="{planning_goal}" path="{path}"
      candidate_path="{candidate_path}"
      retry_hz="10.0">
      <TestGate success="true"/>
      <TestPlan goal="{planning_goal}" path="{candidate_path}" mode="success"/>
      <TestGate success="true"/>
    </GlobalPlanRetry>
  </BehaviorTree>
</root>)";

const char * kBlockedTree = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <GlobalPlanRetry goal="{goal}" planning_goal="{planning_goal}" path="{path}"
      candidate_path="{candidate_path}"
      retry_hz="10.0">
      <TestGate success="false"/>
      <TestPlan goal="{planning_goal}" path="{candidate_path}" mode="success"/>
      <TestGate success="true"/>
    </GlobalPlanRetry>
  </BehaviorTree>
</root>)";

const char * kRestartOutcomeTree = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <GlobalPlanRetry goal="{goal}" planning_goal="{planning_goal}" path="{path}"
      candidate_path="{candidate_path}" retry_hz="10.0">
      <TestGate success="true"/>
      <TestPlan goal="{planning_goal}" path="{candidate_path}" mode="restart"/>
      <TestGate success="true"/>
    </GlobalPlanRetry>
  </BehaviorTree>
</root>)";

struct TestContext
{
  TestContext()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("global_plan_retry_bt_test");
    node->declare_parameter<std::string>("global_frame", "map");
    goal_status = node->create_publisher<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose_internal/_action/status", rclcpp::QoS(10).reliable());
    follow_path_ack = node->create_publisher<std_msgs::msg::UInt64>(
      "/navigation/follow_path_attempt_ack", rclcpp::QoS(10).reliable());
    follow_path_attempt = node->create_subscription<std_msgs::msg::UInt64>(
      "/navigation/follow_path_attempt", rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::UInt64::ConstSharedPtr msg) {
        ++follow_path_attempt_count;
        std_msgs::msg::UInt64 ack;
        ack.data = msg->data;
        follow_path_ack->publish(ack);
      });
    factory.registerFromPlugin(GLOBAL_PLAN_RETRY_BT_PLUGIN);
    factory.registerFromPlugin(FOLLOW_PATH_RETRY_BT_PLUGIN);
    factory.registerFromPlugin(NAV2_PIPELINE_SEQUENCE_BT_PLUGIN);
    factory.registerNodeType<TestGate>("TestGate");
    factory.registerNodeType<TestFailure>("TestFailure");
    factory.registerNodeType<TestPlan>("TestPlan");
    factory.registerNodeType<TestCollisionCheck>("PathCollisionFree");
    factory.registerNodeType<TestPlan>("ComputePathToPose");
    factory.registerNodeType<TestFollowPath>("FollowPath");
    blackboard = BT::Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", node);
    blackboard->set<std::uint64_t>("plan_attempt_count", 0);
    blackboard->set("goal", makeGoal(10, 20));
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = static_cast<builtin_interfaces::msg::Time>(node->now());
    path.poses.push_back(makeGoal(10, 20));
    blackboard->set("path", path);
  }

  static geometry_msgs::msg::PoseStamped makeGoal(const std::int32_t sec, const std::uint32_t nanosec)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp.sec = sec;
    goal.header.stamp.nanosec = nanosec;
    goal.pose.position.x = 2.0;
    goal.pose.position.y = 1.0;
    goal.pose.orientation.w = 1.0;
    return goal;
  }

  void publishActiveGoal(const std::uint8_t id = 0x5a)
  {
    action_msgs::msg::GoalStatusArray status;
    action_msgs::msg::GoalStatus entry;
    entry.goal_info.goal_id.uuid[0] = id;
    entry.goal_info.stamp = static_cast<builtin_interfaces::msg::Time>(node->now());
    entry.status = action_msgs::msg::GoalStatus::STATUS_EXECUTING;
    status.status_list.push_back(entry);
    goal_status->publish(status);
  }

  BT::Tree makeTree(const char * xml)
  {
    return factory.createTreeFromText(xml, blackboard);
  }

  BT::Tree makeProductionTree()
  {
    return factory.createTreeFromFile(BRINGUP_MAP_NAV_TREE_PATH, blackboard);
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<action_msgs::msg::GoalStatusArray>::SharedPtr goal_status;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr follow_path_ack;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr follow_path_attempt;
  std::uint64_t follow_path_attempt_count{0};
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr blackboard;
};
}  // namespace

TEST(GlobalPlanRetryBT, BuildsAndWritesTheValidatedCandidatePath)
{
  TestContext context;
  auto tree = context.makeTree(kSuccessTree);
  context.publishActiveGoal();

  auto result = BT::NodeStatus::RUNNING;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (result == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
    result = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
  const auto path = context.blackboard->get<nav_msgs::msg::Path>("path");
  ASSERT_EQ(path.poses.size(), 1U);
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 2.0);
  const auto planning_goal =
    context.blackboard->get<geometry_msgs::msg::PoseStamped>("planning_goal");
  EXPECT_EQ(planning_goal.header.frame_id, "map");
  EXPECT_EQ(planning_goal.pose.position.x, 2.0);
  EXPECT_NE(planning_goal.header.stamp.sec, 10);
}

TEST(GlobalPlanRetryBT, KeepsTheTreeRunningPastTwentySecondsOfBlockedChecks)
{
  TestContext context;
  auto tree = context.makeTree(kBlockedTree);
  context.publishActiveGoal();

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(21);
  while (std::chrono::steady_clock::now() < deadline) {
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
}

TEST(GlobalPlanRetryBT, AcceptsMatchingOutcomeFromRestartedPlannerInstance)
{
  TestContext context;
  const char * xml = R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <GlobalPlanRetry goal="{goal}" planning_goal="{planning_goal}" path="{path}"
        candidate_path="{candidate_path}"
        retry_hz="10.0">
        <TestGate success="true"/>
        <TestPlan goal="{planning_goal}" path="{candidate_path}" mode="restart"/>
        <TestGate success="true"/>
      </GlobalPlanRetry>
    </BehaviorTree>
  </root>)";
  auto tree = context.makeTree(xml);
  context.publishActiveGoal();
  auto result = BT::NodeStatus::RUNNING;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (result != BT::NodeStatus::SUCCESS && std::chrono::steady_clock::now() < deadline) {
    result = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST(GlobalPlanRetryBT, RejectsOutcomeForSameCoordinatesAndOldTargetStamp)
{
  TestContext context;
  const char * xml = R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <GlobalPlanRetry goal="{goal}" planning_goal="{planning_goal}" path="{path}"
        candidate_path="{candidate_path}"
        retry_hz="10.0">
        <TestGate success="true"/>
        <TestPlan goal="{planning_goal}" path="{candidate_path}" mode="stale"/>
        <TestGate success="true"/>
      </GlobalPlanRetry>
    </BehaviorTree>
  </root>)";
  auto tree = context.makeTree(xml);
  context.publishActiveGoal();
  auto result = BT::NodeStatus::RUNNING;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (result == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
    result = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST(GlobalPlanRetryBT, RejectsLateDuplicateOutcomeForIdenticalTarget)
{
  TestContext context;
  const char * xml = R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <GlobalPlanRetry goal="{goal}" planning_goal="{planning_goal}" path="{path}"
        candidate_path="{candidate_path}"
        retry_hz="10.0">
        <TestGate success="true"/>
        <TestPlan goal="{planning_goal}" path="{candidate_path}" mode="late"/>
        <TestGate success="true"/>
      </GlobalPlanRetry>
    </BehaviorTree>
  </root>)";
  auto tree = context.makeTree(xml);
  context.publishActiveGoal();
  auto result = BT::NodeStatus::RUNNING;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (result == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
    result = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST(GlobalPlanRetryBT, AcceptsMatchingOutcomeAcrossSequenceGapAndPlannerRestart)
{
  TestContext context;
  const char * xml = R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <GlobalPlanRetry goal="{goal}" planning_goal="{planning_goal}" path="{path}"
        candidate_path="{candidate_path}"
        retry_hz="10.0">
        <TestGate success="true"/>
        <TestPlan goal="{planning_goal}" path="{candidate_path}" mode="sequence_gap"/>
        <TestGate success="true"/>
      </GlobalPlanRetry>
    </BehaviorTree>
  </root>)";
  auto tree = context.makeTree(xml);
  context.publishActiveGoal();
  auto result = BT::NodeStatus::RUNNING;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (result == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
    result = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST(GlobalPlanRetryBT, IsolatesConcurrentSamePoseRequestsByRequestIdentity)
{
  TestContext context;
  context.publishActiveGoal();
  const auto make_blackboard = [&context]() {
      auto blackboard = BT::Blackboard::create();
      blackboard->set<rclcpp::Node::SharedPtr>("node", context.node);
      blackboard->set("goal", TestContext::makeGoal(10, 20));
      blackboard->set<std::uint64_t>("plan_attempt_count", 0);
      return blackboard;
    };
  auto first_blackboard = make_blackboard();
  auto second_blackboard = make_blackboard();
  auto first_tree = context.factory.createTreeFromText(kRestartOutcomeTree, first_blackboard);
  auto second_tree = context.factory.createTreeFromText(kRestartOutcomeTree, second_blackboard);

  auto first_result = BT::NodeStatus::RUNNING;
  auto second_result = BT::NodeStatus::RUNNING;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while ((first_result != BT::NodeStatus::SUCCESS ||
    second_result != BT::NodeStatus::SUCCESS) &&
    std::chrono::steady_clock::now() < deadline)
  {
    first_result = first_tree.tickRoot();
    second_result = second_tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(first_result, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(second_result, BT::NodeStatus::SUCCESS);
  const auto first_request =
    first_blackboard->get<geometry_msgs::msg::PoseStamped>("planning_goal");
  const auto second_request =
    second_blackboard->get<geometry_msgs::msg::PoseStamped>("planning_goal");
  EXPECT_EQ(first_request.pose.position.x, second_request.pose.position.x);
  EXPECT_TRUE(first_request.header.stamp.sec != second_request.header.stamp.sec ||
    first_request.header.stamp.nanosec != second_request.header.stamp.nanosec);
}

TEST(GlobalPlanRetryBT, RejectsOutcomeWhoseRequestIdentityDoesNotMatch)
{
  TestContext context;
  const char * xml = R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <GlobalPlanRetry goal="{goal}" planning_goal="{planning_goal}" path="{path}"
        candidate_path="{candidate_path}" retry_hz="10.0">
        <TestGate success="true"/>
        <TestPlan goal="{planning_goal}" path="{candidate_path}" mode="wrong_request"/>
        <TestGate success="true"/>
      </GlobalPlanRetry>
    </BehaviorTree>
  </root>)";
  auto tree = context.makeTree(xml);
  context.publishActiveGoal();
  auto result = BT::NodeStatus::RUNNING;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (result == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
    result = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(result, BT::NodeStatus::FAILURE);
}

TEST(GlobalPlanRetryBT, WaitsForTemporaryGoalCoverageShortfallAndRechecks)
{
  TestContext context;
  const char * xml = R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <GlobalPlanRetry goal="{goal}" planning_goal="{planning_goal}" path="{path}"
        candidate_path="{candidate_path}" retry_hz="10.0">
        <TestGate success="true"/>
        <TestPlan goal="{planning_goal}" path="{candidate_path}" mode="out_of_bounds"/>
        <TestGate success="true"/>
      </GlobalPlanRetry>
    </BehaviorTree>
  </root>)";
  auto tree = context.makeTree(xml);
  context.publishActiveGoal();
  auto result = BT::NodeStatus::RUNNING;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (result != BT::NodeStatus::SUCCESS && std::chrono::steady_clock::now() < deadline) {
    result = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);
  EXPECT_GT(context.blackboard->get<std::uint64_t>("plan_attempt_count"), 1U);
}

TEST(GlobalPlanRetryBT, WaitsForBlockedCandidateAndResumesAfterItClears)
{
  TestContext context;
  context.blackboard->set("candidate_clear", false);
  const char * xml = R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <GlobalPlanRetry goal="{goal}" planning_goal="{planning_goal}" path="{path}"
        candidate_path="{candidate_path}"
        retry_hz="10.0">
        <TestGate success="true"/>
        <TestPlan goal="{planning_goal}" path="{candidate_path}" mode="success"/>
        <TestGate success="{candidate_clear}"/>
      </GlobalPlanRetry>
    </BehaviorTree>
  </root>)";
  auto tree = context.makeTree(xml);
  context.publishActiveGoal();

  auto result = BT::NodeStatus::RUNNING;
  for (int i = 0; i < 10; ++i) {
    result = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_EQ(result, BT::NodeStatus::RUNNING);
  EXPECT_TRUE(context.blackboard->get<nav_msgs::msg::Path>("path").poses.empty());
  context.blackboard->set("candidate_clear", true);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (result != BT::NodeStatus::SUCCESS && std::chrono::steady_clock::now() < deadline) {
    result = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);
}

TEST(GlobalPlanRetryBT, ProductionTreeWaitsForCandidateRecoveryThenExecutes)
{
  TestContext context;
  context.blackboard->set("candidate_clear", false);
  auto tree = context.makeProductionTree();
  context.publishActiveGoal();

  auto result = BT::NodeStatus::RUNNING;
  const auto blocked_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < blocked_deadline) {
    rclcpp::spin_some(context.node);
    result = tree.tickRoot();
    if (context.blackboard->get<std::uint64_t>("plan_attempt_count") > 0 &&
      context.blackboard->get<nav_msgs::msg::Path>("path").poses.empty()) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(result, BT::NodeStatus::RUNNING);
  EXPECT_GT(context.blackboard->get<std::uint64_t>("plan_attempt_count"), 0U);
  EXPECT_TRUE(context.blackboard->get<nav_msgs::msg::Path>("path").poses.empty());

  context.blackboard->set("candidate_clear", true);
  const auto recovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (result != BT::NodeStatus::SUCCESS &&
    std::chrono::steady_clock::now() < recovery_deadline)
  {
    rclcpp::spin_some(context.node);
    result = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(context.blackboard->get<nav_msgs::msg::Path>("path").poses.empty());
}

TEST(GlobalPlanRetryBT, NewActionWithRepeatedTargetStartsANewPlan)
{
  TestContext context;
  auto tree = context.makeTree(kSuccessTree);
  context.publishActiveGoal(0x5a);
  auto result = BT::NodeStatus::RUNNING;
  const auto first_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (result != BT::NodeStatus::SUCCESS && std::chrono::steady_clock::now() < first_deadline) {
    result = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(result, BT::NodeStatus::SUCCESS);
  const auto first_planning_goal =
    context.blackboard->get<geometry_msgs::msg::PoseStamped>("planning_goal");

  context.publishActiveGoal(0x5b);
  result = BT::NodeStatus::RUNNING;
  const auto replacement_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (result != BT::NodeStatus::SUCCESS &&
    std::chrono::steady_clock::now() < replacement_deadline)
  {
    result = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);
  const auto second_planning_goal =
    context.blackboard->get<geometry_msgs::msg::PoseStamped>("planning_goal");
  EXPECT_EQ(first_planning_goal.pose.position.x, second_planning_goal.pose.position.x);
  EXPECT_TRUE(first_planning_goal.header.stamp.sec != second_planning_goal.header.stamp.sec ||
    first_planning_goal.header.stamp.nanosec != second_planning_goal.header.stamp.nanosec);
  const auto path = context.blackboard->get<nav_msgs::msg::Path>("path");
  ASSERT_EQ(path.poses.size(), 1U);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 2.0);
}

TEST(FollowPathRetryBT, HoldsTaskPastTwentySecondsOfRepeatedLocalBlockage)
{
  TestContext context;
  const char * xml = R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <FollowPathRetry goal="{goal}" path="{path}">
        <TestFailure/>
        <TestGate success="true"/>
      </FollowPathRetry>
    </BehaviorTree>
  </root>)";
  auto tree = context.makeTree(xml);
  context.publishActiveGoal();

  auto result = BT::NodeStatus::RUNNING;
  const auto setup_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < setup_deadline) {
    rclcpp::spin_some(context.node);
    result = tree.tickRoot();
    if (context.blackboard->get<nav_msgs::msg::Path>("path").poses.empty()) {
      auto path = context.blackboard->get<nav_msgs::msg::Path>("path");
      path.header.stamp = static_cast<builtin_interfaces::msg::Time>(context.node->now());
      path.poses.push_back(TestContext::makeGoal(10, 20));
      context.blackboard->set("path", path);
    }
    if (result == BT::NodeStatus::RUNNING && context.follow_path_attempt_count > 0 &&
      !context.blackboard->get<nav_msgs::msg::Path>("path").poses.empty()) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(result, BT::NodeStatus::RUNNING);

  const auto blocked_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(21);
  while (std::chrono::steady_clock::now() < blocked_deadline) {
    rclcpp::spin_some(context.node);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  tree.haltTree();
  EXPECT_EQ(tree.rootNode()->status(), BT::NodeStatus::IDLE);
}
