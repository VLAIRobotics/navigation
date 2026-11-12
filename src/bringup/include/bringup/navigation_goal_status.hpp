#pragma once

#include <string>
#include <utility>

namespace bringup
{

class NavigationGoalStatusText
{
public:
  void beginGoal(std::string text)
  {
    waiting_for_planner_ = true;
    text_ = std::move(text);
  }

  void plannerResponded()
  {
    waiting_for_planner_ = false;
  }

  void showRejection(std::string text)
  {
    waiting_for_planner_ = false;
    text_ = std::move(text);
  }

  void showStatus(std::string text)
  {
    waiting_for_planner_ = false;
    text_ = std::move(text);
  }

  bool finishIfWaiting(std::string text)
  {
    if (!waiting_for_planner_) return false;
    waiting_for_planner_ = false;
    text_ = std::move(text);
    return true;
  }

  bool waitingForPlanner() const
  {
    return waiting_for_planner_;
  }

  const std::string & text() const
  {
    return text_;
  }

private:
  bool waiting_for_planner_{false};
  std::string text_;
};

}  // namespace bringup
