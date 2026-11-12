#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QShortcut>
#include <QKeySequence>
#include <QRegularExpression>
#include <QTime>
#include <QTimer>
#include <QVBoxLayout>

#include <action_msgs/msg/goal_status_array.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "bringup/goal_rejection_cache.hpp"
#include "bringup/navigation_goal_status.hpp"

namespace bringup
{

class NavigationStatusPanel : public rviz_common::Panel
{
public:
  explicit NavigationStatusPanel(QWidget * parent = nullptr)
  : rviz_common::Panel(parent)
  {
    auto * layout = new QVBoxLayout(this);
    navigation_ = new QLabel("导航：等待状态", this);
    execution_ = new QLabel("任务执行：等待导航任务", this);
    health_ = new QLabel("定位：等待健康状态", this);
    obstacles_ = new QLabel("实时障碍：等待数据", this);
    planner_ = new QLabel("规划：等待状态", this);
    goal_manager_ = new QLabel("目标管理：等待目标", this);
    exploration_ = new QLabel("前沿探索：未启用", this);
    for (auto * label : {navigation_, execution_, goal_manager_, health_, obstacles_, planner_,
      exploration_})
    {
      label->setWordWrap(true);
      layout->addWidget(label);
    }
    save_map_button_ = new QPushButton("保存融合地图 (Ctrl+R)", this);
    save_map_status_ = new QLabel("地图保存：等待操作", this);
    save_map_status_->setWordWrap(true);
    layout->addWidget(save_map_button_);
    layout->addWidget(save_map_status_);
    connect(save_map_button_, &QPushButton::clicked, this, [this]() {saveMap();});
    auto * save_map_shortcut = new QShortcut(QKeySequence("Ctrl+R"), this);
    connect(save_map_shortcut, &QShortcut::activated, this, [this]() {saveMap();});
    auto * legend = new QLabel(
      "蓝到红：膨胀代价；青色：机器人内切禁区；紫色：致命障碍\n"
      "黄线：全局路径；绿线：局部路径", this);
    legend->setWordWrap(true);
    layout->addWidget(legend);
    auto * heading = new QLabel("最近警告 / 错误（历史记录）", this);
    layout->addWidget(heading);
    messages_ = new QPlainTextEdit(this);
    messages_->setReadOnly(true);
    messages_->setMaximumBlockCount(40);
    messages_->setMinimumHeight(85);
    messages_->setMaximumHeight(180);
    messages_->setPlaceholderText("暂无诊断消息。规划失败时显示节点原始原因。");
    layout->addWidget(messages_);

    node_ = std::make_shared<rclcpp::Node>("navigation_status_panel",
      rclcpp::NodeOptions().use_global_arguments(false));
    save_map_client_ = node_->create_client<std_srvs::srv::Trigger>("/navigation/save_map");
    executor_.add_node(node_);
    ready_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/navigation_ready", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        navigation_ready_ = message->data;
        if (navigation_ready_) {
          last_navigation_pause_reason_.clear();
          navigation_->setText("导航门控：已就绪");
        } else {
          showNavigationPauseReason();
        }
      });
    health_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/fastlio/health", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        have_health_ = true;
        healthy_ = message->data;
        health_received_ = Clock::now();
      });
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/obstacles/blocking", rclcpp::QoS(1).best_effort(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        have_cloud_ = true;
        point_count_ = static_cast<std::uint64_t>(message->width) * message->height;
        cloud_received_ = Clock::now();
      });
    planner_sub_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/compute_path_to_pose/_action/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr message) {
        if (message->status_list.empty()) return;
        auto latest = message->status_list.end();
        for (auto it = message->status_list.begin(); it != message->status_list.end(); ++it) {
          const auto & stamp = it->goal_info.stamp;
          if (have_navigation_goal_id_ && stampLess(stamp, active_navigation_goal_stamp_)) {
            continue;
          }
          if (latest == message->status_list.end() ||
            stampLess(latest->goal_info.stamp, stamp))
          {
            latest = it;
          }
        }
        if (latest == message->status_list.end()) return;

        const auto & stamp = latest->goal_info.stamp;
        if (have_planner_status_ && stampLess(stamp, latest_planner_status_stamp_)) return;
        have_planner_status_ = true;
        planner_display_state_.plannerResponded();
        latest_planner_status_stamp_ = stamp;
        latest_planner_status_ = latest->status;
        if (!have_current_goal_rejection_) {
          showPlannerStatus(latest_planner_status_);
        }
      });
    navigation_goal_sub_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr message) {
        if (message->status_list.empty()) return;
        const auto latest = std::max_element(
          message->status_list.begin(), message->status_list.end(),
          [](const auto & a, const auto & b) {
            if (a.goal_info.stamp.sec != b.goal_info.stamp.sec) {
              return a.goal_info.stamp.sec < b.goal_info.stamp.sec;
            }
            return a.goal_info.stamp.nanosec < b.goal_info.stamp.nanosec;
          });
        const auto & goal_id = latest->goal_info.goal_id.uuid;
        if (!have_navigation_goal_id_ || goal_id != active_navigation_goal_id_) {
          active_navigation_goal_id_ = goal_id;
          have_navigation_goal_id_ = true;
          active_navigation_goal_stamp_ = latest->goal_info.stamp;
          local_wait_status_.clear();
          messages_->clear();
          active_execution_goal_id_.clear();
          have_navigation_status_ = false;
          latest_navigation_status_stamp_ = builtin_interfaces::msg::Time();
          have_planner_status_ = false;
          have_current_goal_rejection_ = false;
          latest_planner_status_stamp_ = builtin_interfaces::msg::Time();
          const QString waiting_text = "导航目标已更新，等待本目标规划结果";
          planner_display_state_.beginGoal(waiting_text.toStdString());
          planner_->setText(QString::fromStdString(planner_display_state_.text()));
          updateActiveExecutionGoalIdentity();
          updateActiveGoalPoseIdentity();
        }
        if (!have_navigation_status_ ||
          !stampLess(latest->goal_info.stamp, latest_navigation_status_stamp_))
        {
          have_navigation_status_ = true;
          latest_navigation_status_stamp_ = latest->goal_info.stamp;
          latest_navigation_status_ = latest->status;
          finishPlannerWaitIfNavigationEnded(latest_navigation_status_);
          refreshExecutionStatus();
        }
      });
    goal_identity_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/navigation/goal_identity", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        const auto first_separator = message->data.find(':');
        const auto second_separator = first_separator == std::string::npos ?
          std::string::npos : message->data.find(':', first_separator + 1);
        if (first_separator == std::string::npos || second_separator == std::string::npos ||
          first_separator == 0 || second_separator - first_separator != 33 ||
          message->data.size() - second_separator != 33)
        {
          mapped_public_goal_id_.clear();
          mapped_execution_goal_id_.clear();
          active_execution_goal_id_.clear();
          updateActiveGoalPoseIdentity();
          refreshExecutionStatus();
          return;
        }
        mapped_public_goal_id_ = message->data.substr(first_separator + 1, 32);
        mapped_execution_goal_id_ = message->data.substr(second_separator + 1, 32);
        updateActiveExecutionGoalIdentity();
        updateActiveGoalPoseIdentity();
        refreshExecutionStatus();
      });
    goal_pose_identity_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/navigation/goal_pose_identity", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        setMappedGoalPoseIdentity(message->data);
        updateActiveGoalPoseIdentity();
      });
    local_wait_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/navigation/local_wait_status", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        local_wait_status_ = message->data;
        refreshExecutionStatus();
      });
    exploration_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/navigation/frontier_exploration_status", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        have_exploration_status_ = true;
        exploration_->setText("前沿探索：" + QString::fromStdString(message->data));
      });
    goal_manager_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/navigation/goal_manager_status", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        goal_manager_->setText("目标管理：" + QString::fromStdString(message->data));
      });
    log_sub_ = node_->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", rclcpp::QoS(50).best_effort().durability_volatile(),
      [this](rcl_interfaces::msg::Log::ConstSharedPtr message) {
        if (message->level < rcl_interfaces::msg::Log::WARN) return;
        const std::vector<std::string> sources = {
          "planner_server", "controller_server", "bt_navigator",
          "bt_navigator_navigate_to_pose_rclcpp_node", "navigation_gate",
          "global_costmap", "local_costmap", "nonground_height_filter",
          "navigation_obstacle_display", "fastdem"};
        if (std::none_of(sources.begin(), sources.end(), [&](const auto & source) {
            return message->name == source || message->name.find(source + ".") == 0;
          })) return;
        const auto text = QString::fromStdString(message->msg);
        if (isGoalRejectionDiagnostic(text)) {
          if (!showGoalRejectionDiagnostic(text)) return;
          return;
        }
        if (have_navigation_goal_id_ && stampLess(message->stamp, active_navigation_goal_stamp_)) {
          return;
        }
        QString hint;
        if (message->name == "navigation_gate" &&
          text.startsWith("Navigation gate PAUSED:"))
        {
          if (!navigation_ready_) {
            last_navigation_pause_reason_ = text.mid(
              QStringLiteral("Navigation gate PAUSED:").size()).trimmed();
            showNavigationPauseReason();
          }
        }
        if (text.startsWith("Goal is occupied in inflated global costmap:")) {
          hint = "本条诊断中的目标坐标落在致命代价格（254）；请重新选择该目标。\n";
        } else if (text.startsWith("Goal is in unknown space:")) {
          hint = "目标格 255 表示 NO_INFORMATION；恢复可尝试观测该格，但规划器仍拒绝穿越未知区域。\n";
        } else if (text.startsWith("Goal cost exceeds recovery threshold:")) {
          hint = "恢复入口目标代价达到致命阈值 254，恢复已停止。\n";
        } else if (text.startsWith("Cannot plan:")) {
          hint = "起点或终点不满足规划条件。原始代价：253=膨胀禁区，254=障碍，255=未知。\n";
        } else if (text.startsWith("No connected path:")) {
          hint = "未找到连通路径，请检查膨胀代价、障碍和未知区域。\n";
        } else if (text.contains("outside map bounds")) {
          hint = "起点或终点超出地图边界。\n";
        }
        messages_->appendPlainText(QTime::currentTime().toString("HH:mm:ss") + " [" +
          QString::fromStdString(message->name) + "]\n" + hint + text);
      });
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this]() {
      if (!rclcpp::ok()) return;
      executor_.spin_some(std::chrono::milliseconds(5));
      refresh();
    });
    timer_->start(100);
  }

  ~NavigationStatusPanel() override
  {
    timer_->stop();
    executor_.remove_node(node_);
  }

private:
  using Clock = std::chrono::steady_clock;

  static bool stampLess(
    const builtin_interfaces::msg::Time & lhs,
    const builtin_interfaces::msg::Time & rhs)
  {
    return lhs.sec < rhs.sec || (lhs.sec == rhs.sec && lhs.nanosec < rhs.nanosec);
  }

  static QString statusField(const std::string & status, const std::string & key)
  {
    const auto start = status.find(key);
    if (start == std::string::npos) {
      return {};
    }
    const auto value_start = start + key.size();
    const auto end = status.find(';', value_start);
    return QString::fromStdString(status.substr(value_start, end - value_start));
  }

  static std::string uuidText(const std::array<std::uint8_t, 16> & uuid)
  {
    constexpr char kHex[] = "0123456789abcdef";
    std::string text;
    text.reserve(uuid.size() * 2);
    for (const auto byte : uuid) {
      text.push_back(kHex[(byte >> 4) & 0x0f]);
      text.push_back(kHex[byte & 0x0f]);
    }
    return text;
  }

  void setMappedGoalPoseIdentity(const std::string & data)
  {
    std::vector<std::string> fields;
    std::istringstream stream(data);
    std::string field;
    while (std::getline(stream, field, ':')) fields.push_back(field);
    if (fields.size() != 6 || fields[0].size() != 32 || fields[1].size() != 32) {
      have_mapped_goal_pose_identity_ = false;
      mapped_goal_pose_public_id_.clear();
      mapped_goal_pose_execution_id_.clear();
      return;
    }
    try {
      const auto stamp_sec = std::stoll(fields[2]);
      const auto stamp_nanosec = std::stoul(fields[3]);
      const auto goal_x = std::stod(fields[4]);
      const auto goal_y = std::stod(fields[5]);
      if (stamp_sec < std::numeric_limits<std::int32_t>::min() ||
        stamp_sec > std::numeric_limits<std::int32_t>::max() || stamp_nanosec >= 1000000000UL ||
        !std::isfinite(goal_x) || !std::isfinite(goal_y))
      {
        throw std::invalid_argument("invalid goal pose identity values");
      }
      mapped_goal_pose_public_id_ = fields[0];
      mapped_goal_pose_execution_id_ = fields[1];
      mapped_goal_pose_stamp_.sec = static_cast<std::int32_t>(stamp_sec);
      mapped_goal_pose_stamp_.nanosec = static_cast<std::uint32_t>(stamp_nanosec);
      mapped_goal_pose_x_ = goal_x;
      mapped_goal_pose_y_ = goal_y;
      have_mapped_goal_pose_identity_ = true;
    } catch (const std::exception &) {
      have_mapped_goal_pose_identity_ = false;
      mapped_goal_pose_public_id_.clear();
      mapped_goal_pose_execution_id_.clear();
    }
  }

  void updateActiveGoalPoseIdentity()
  {
    have_active_goal_pose_ = have_navigation_goal_id_ && have_mapped_goal_pose_identity_ &&
      mapped_goal_pose_public_id_ == uuidText(active_navigation_goal_id_) &&
      mapped_goal_pose_execution_id_ == mapped_execution_goal_id_ &&
      !mapped_execution_goal_id_.empty();
    if (!have_active_goal_pose_) return;
    active_goal_pose_stamp_ = mapped_goal_pose_stamp_;
    active_goal_x_ = mapped_goal_pose_x_;
    active_goal_y_ = mapped_goal_pose_y_;
    const auto pending = goal_rejection_cache_.takeForGoal(
      active_goal_pose_stamp_.sec, active_goal_pose_stamp_.nanosec);
    for (const auto & diagnostic : pending) {
      showGoalRejectionDiagnostic(QString::fromStdString(diagnostic.message));
    }
  }

  static bool isGoalRejectionDiagnostic(const QString & text)
  {
    return text.contains("category=static_goal_") ||
           text.contains("category=global_no_path ") ||
           text.contains("category=global_start_or_goal_invalid ") ||
           text.startsWith("Goal is in unknown space:") ||
           text.startsWith("Goal is outside the current inflated global costmap:") ||
           text.startsWith("Goal cost exceeds recovery threshold:");
  }

  static bool diagnosticGoalStamp(
    const QString & text, builtin_interfaces::msg::Time & stamp)
  {
    const QRegularExpression expression("(?:^|\\s)goal_stamp=(\\d+)\\.(\\d+)");
    const auto match = expression.match(text);
    if (!match.hasMatch()) return false;
    bool seconds_ok = false;
    bool nanoseconds_ok = false;
    const auto seconds = match.captured(1).toLongLong(&seconds_ok);
    const auto nanoseconds = match.captured(2).toUInt(&nanoseconds_ok);
    if (!seconds_ok || !nanoseconds_ok ||
      seconds < std::numeric_limits<std::int32_t>::min() ||
      seconds > std::numeric_limits<std::int32_t>::max() || nanoseconds >= 1000000000U)
    {
      return false;
    }
    stamp.sec = static_cast<std::int32_t>(seconds);
    stamp.nanosec = nanoseconds;
    return true;
  }

  bool diagnosticMatchesActiveGoal(const QString & text) const
  {
    if (!have_active_goal_pose_) return false;
    builtin_interfaces::msg::Time stamp;
    return diagnosticGoalStamp(text, stamp) &&
           stamp.sec == active_goal_pose_stamp_.sec &&
           stamp.nanosec == active_goal_pose_stamp_.nanosec;
  }

  bool showGoalRejectionDiagnostic(const QString & text)
  {
    if (!have_active_goal_pose_ || !diagnosticMatchesActiveGoal(text)) {
      builtin_interfaces::msg::Time stamp;
      if (!diagnosticGoalStamp(text, stamp)) return false;
      return goal_rejection_cache_.remember(
        stamp.sec, stamp.nanosec, text.toStdString());
    }
    const auto summary = goalRejectionSummary(text);
    if (summary.isEmpty()) return false;
    have_current_goal_rejection_ = true;
    const auto goal_summary = QString("目标 (%1, %2)：%3")
      .arg(active_goal_x_, 0, 'f', 3).arg(active_goal_y_, 0, 'f', 3).arg(summary);
    planner_display_state_.showRejection(goal_summary.toStdString());
    planner_->setText(QString::fromStdString(planner_display_state_.text()));
    messages_->appendPlainText(QTime::currentTime().toString("HH:mm:ss") +
      " [本目标拒绝原因]\n" + goal_summary + "\n" + text);
    return true;
  }

  static QString goalRejectionSummary(const QString & text)
  {
    if (text.contains("static_goal_unknown") || text.startsWith("Goal is in unknown space:")) {
      return "目标位于未知区域；未知区域限制保持生效。";
    }
    if (text.contains("static_goal_out_of_bounds") ||
      text.startsWith("Goal is outside the current inflated global costmap:"))
    {
      return "目标超出当前全局代价地图边界。";
    }
    if (text.contains("category=global_no_path ")) {
      return "全局地图上没有连通路线。";
    }
    if (text.contains("category=global_start_or_goal_invalid ")) {
      return "起点或目标落在不可通行代价格中。";
    }
    if (text.startsWith("Goal cost exceeds recovery threshold:")) {
      return "目标落在障碍或机器人膨胀安全区。";
    }
    if (text.contains("static_goal_invalid")) {
      if (text.contains("reason=empty_frame") || text.contains("reason=non_finite_pose")) {
        return "目标位姿无效。";
      }
      return "目标落在障碍或机器人膨胀安全区。";
    }
    return {};
  }

  void finishPlannerWaitIfNavigationEnded(std::uint8_t status)
  {
    using Status = action_msgs::msg::GoalStatus;
    QString text;
    switch (status) {
      case Status::STATUS_SUCCEEDED:
        text = "目标已完成；本目标未等待新的规划结果";
        break;
      case Status::STATUS_CANCELED:
        text = "本目标已取消";
        break;
      case Status::STATUS_ABORTED:
        text = "本目标在规划结果返回前失败；请查看本目标拒绝原因";
        break;
      default:
        return;
    }
    if (!planner_display_state_.finishIfWaiting(text.toStdString())) return;
    planner_->setText(QString::fromStdString(planner_display_state_.text()));
  }

  void updateActiveExecutionGoalIdentity()
  {
    if (have_navigation_goal_id_ &&
      mapped_public_goal_id_ == uuidText(active_navigation_goal_id_))
    {
      active_execution_goal_id_ = mapped_execution_goal_id_;
    } else {
      active_execution_goal_id_.clear();
    }
  }

  bool localStatusMatchesCurrentGoal() const
  {
    return !active_execution_goal_id_.empty() &&
      statusField(local_wait_status_, "goal_id=").toStdString() == active_execution_goal_id_;
  }

  void refreshExecutionStatus()
  {
    using Status = action_msgs::msg::GoalStatus;
    if (have_navigation_status_ && latest_navigation_status_ == Status::STATUS_SUCCEEDED) {
      execution_->setText("任务已到达");
      return;
    }
    if (have_navigation_status_ && latest_navigation_status_ == Status::STATUS_ABORTED) {
      execution_->setText("任务失败，请查看最后碰撞原因");
      return;
    }
    if (have_navigation_status_ && latest_navigation_status_ == Status::STATUS_CANCELED) {
      execution_->setText("任务已取消");
      return;
    }
    if (localStatusMatchesCurrentGoal() &&
      local_wait_status_.find("state=blocked_waiting_retry") == 0)
    {
      const auto remaining = statusField(local_wait_status_, "remaining_sec=");
      const auto collision = statusField(local_wait_status_, "last_collision=");
      QString text = remaining.isEmpty() ? "导航受阻等待，通路恢复后继续" :
        QString("导航受阻等待，剩余 %1 秒").arg(remaining);
      if (!collision.isEmpty()) {
        text += "\n最后碰撞：" + collision;
      }
      execution_->setText(text);
      return;
    }
    if (localStatusMatchesCurrentGoal() && local_wait_status_.find("state=failed") == 0) {
      const auto reason = statusField(local_wait_status_, "reason=");
      const auto collision = statusField(local_wait_status_, "last_collision=");
      QString text = "局部重试失败：" + reason;
      if (!collision.isEmpty()) {
        text += "\n最后碰撞：" + collision;
      }
      execution_->setText(text);
      return;
    }
    if (localStatusMatchesCurrentGoal() && local_wait_status_.find("state=retrying") == 0) {
      execution_->setText("局部路径恢复中，正在重试");
      return;
    }
    if (localStatusMatchesCurrentGoal() && local_wait_status_.find("state=planning") == 0) {
      execution_->setText("等待当前目标的新路径");
      return;
    }
    if (localStatusMatchesCurrentGoal() && local_wait_status_.find("state=executing") == 0) {
      execution_->setText("局部路径执行中");
      return;
    }
    if (localStatusMatchesCurrentGoal() && local_wait_status_.find("state=stopped") == 0) {
      execution_->setText("局部执行已停止，等待导航终态");
      return;
    }
    if (!have_navigation_status_) {
      execution_->setText("任务执行：等待导航任务");
      return;
    }
    switch (latest_navigation_status_) {
      case Status::STATUS_ACCEPTED:
      case Status::STATUS_EXECUTING:
        execution_->setText("任务执行中");
        break;
      case Status::STATUS_SUCCEEDED:
        execution_->setText("任务已到达");
        break;
      case Status::STATUS_ABORTED:
        execution_->setText("任务失败，请查看最后碰撞原因");
        break;
      case Status::STATUS_CANCELED:
        execution_->setText("任务已取消");
        break;
      default:
        execution_->setText("任务执行：等待导航任务");
        break;
    }
  }

  void showPlannerStatus(std::uint8_t status)
  {
    using Status = action_msgs::msg::GoalStatus;
    switch (status) {
      case Status::STATUS_ACCEPTED:
      case Status::STATUS_EXECUTING:
        setPlannerStatusText("规划：正在计算路径");
        break;
      case Status::STATUS_SUCCEEDED:
        setPlannerStatusText("最近一次规划：成功");
        break;
      case Status::STATUS_ABORTED:
        setPlannerStatusText("最近一次规划：失败，请查看下方目标关联诊断");
        break;
      case Status::STATUS_CANCELED:
        setPlannerStatusText("最近一次规划：已取消");
        break;
      default:
        setPlannerStatusText("规划：等待状态");
        break;
    }
  }

  void setPlannerStatusText(const QString & text)
  {
    planner_display_state_.showStatus(text.toStdString());
    planner_->setText(QString::fromStdString(planner_display_state_.text()));
  }

  QString navigationPauseHint(const QString & reason) const
  {
    if (reason.contains("FAST-LIO", Qt::CaseInsensitive)) {
      return "检查 FAST-LIO 健康状态以及 LiDAR/IMU 输入。";
    }
    if (reason.contains("terrain", Qt::CaseInsensitive) ||
      reason.contains("FastDEM", Qt::CaseInsensitive))
    {
      return "检查 FastDEM 地形状态和数据时效。";
    }
    if (reason.contains("localization", Qt::CaseInsensitive) ||
      reason.contains("relocalization", Qt::CaseInsensitive) ||
      reason.contains("handoff", Qt::CaseInsensitive))
    {
      return "检查重定位状态与 map→odom 交接。";
    }
    return "检查导航门控输入和上方健康状态。";
  }

  void showNavigationPauseReason()
  {
    if (last_navigation_pause_reason_.isEmpty()) {
      navigation_->setText("导航门控：尚未就绪 / 已暂停");
      return;
    }
    navigation_->setText(QString("导航门控：已暂停\n原因：%1\n%2")
      .arg(last_navigation_pause_reason_, navigationPauseHint(last_navigation_pause_reason_)));
  }

  void saveMap()
  {
    using Trigger = std_srvs::srv::Trigger;
    if (!save_map_client_->service_is_ready()) {
      save_map_status_->setText("地图保存：保存服务暂不可用");
      return;
    }
    save_map_button_->setEnabled(false);
    save_map_status_->setText("地图保存：正在收集一致快照并写入文件");
    save_map_client_->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this](rclcpp::Client<Trigger>::SharedFuture future) {
        save_map_button_->setEnabled(true);
        try {
          const auto response = future.get();
          const auto prefix = response->success ? QString("地图保存完成：") : QString("地图保存失败：");
          save_map_status_->setText(prefix + QString::fromStdString(response->message));
        } catch (const std::exception & exception) {
          save_map_status_->setText(QString("地图保存失败：%1").arg(exception.what()));
        }
      });
  }

  void refresh()
  {
    if (ready_sub_->get_publisher_count() == 0) {
      navigation_->setText("导航门控：发布节点离线 / 尚未启动");
    }
    if (have_health_) {
      const double age = std::chrono::duration<double>(Clock::now() - health_received_).count();
      health_->setText(age > 0.5 ? "定位：健康状态已超时" :
        (healthy_ ? "定位：健康" : "定位：未通过健康检查"));
    }
    if (have_cloud_) {
      const double age = std::chrono::duration<double>(Clock::now() - cloud_received_).count();
      if (age > 1.0) {
        obstacles_->setText(QString("实时障碍：已 %1 秒没有更新").arg(age, 0, 'f', 1));
      } else if (point_count_ == 0) {
        obstacles_->setText("实时障碍：已收到空帧，本帧无阻挡点");
      } else {
        obstacles_->setText(QString("实时障碍：收到 %1 点，%2 秒前接收")
          .arg(static_cast<qulonglong>(point_count_)).arg(age, 0, 'f', 1));
      }
    }
    if (exploration_sub_->get_publisher_count() == 0) {
      exploration_->setText("前沿探索：未启用");
    } else if (!have_exploration_status_) {
      exploration_->setText("前沿探索：等待节点状态");
    }
  }

  QLabel * navigation_;
  QLabel * execution_;
  QLabel * health_;
  QLabel * obstacles_;
  QLabel * planner_;
  QLabel * goal_manager_;
  QLabel * exploration_;
  QPushButton * save_map_button_;
  QLabel * save_map_status_;
  QPlainTextEdit * messages_;
  QTimer * timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_, health_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr planner_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr navigation_goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_identity_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_pose_identity_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr local_wait_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr exploration_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_manager_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_map_client_;
  bool have_health_{false}, healthy_{false}, have_cloud_{false}, navigation_ready_{false};
  QString last_navigation_pause_reason_;
  bool have_navigation_goal_id_{false};
  bool have_mapped_goal_pose_identity_{false};
  bool have_active_goal_pose_{false};
  bool have_exploration_status_{false};
  std::array<std::uint8_t, 16> active_navigation_goal_id_{};
  std::string mapped_goal_pose_public_id_, mapped_goal_pose_execution_id_;
  builtin_interfaces::msg::Time mapped_goal_pose_stamp_{}, active_goal_pose_stamp_{};
  double mapped_goal_pose_x_{0.0}, mapped_goal_pose_y_{0.0};
  double active_goal_x_{0.0}, active_goal_y_{0.0};
  builtin_interfaces::msg::Time active_navigation_goal_stamp_{};
  bool have_planner_status_{false};
  bool have_current_goal_rejection_{false};
  builtin_interfaces::msg::Time latest_planner_status_stamp_{};
  std::uint8_t latest_planner_status_{0};
  bool have_navigation_status_{false};
  builtin_interfaces::msg::Time latest_navigation_status_stamp_{};
  std::uint8_t latest_navigation_status_{0};
  std::string mapped_public_goal_id_, mapped_execution_goal_id_, active_execution_goal_id_;
  std::string local_wait_status_;
  bringup::GoalRejectionCache goal_rejection_cache_;
  bringup::NavigationGoalStatusText planner_display_state_;
  std::uint64_t point_count_{0};
  Clock::time_point health_received_, cloud_received_;
};

}  // namespace bringup

PLUGINLIB_EXPORT_CLASS(bringup::NavigationStatusPanel, rviz_common::Panel)
