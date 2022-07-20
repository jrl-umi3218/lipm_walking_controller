#include <mc_rtc/logging.h>

#include <ExternalFootstepPlanner/ExternalFootstepPlanner.h>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{
ExternalFootstepPlanner::~ExternalFootstepPlanner()
{
  deactivate();
}

void ExternalFootstepPlanner::activate()
{
  mc_rtc::log::warning("[{}] activate not implemented in this planner", name());
}

void ExternalFootstepPlanner::deactivate()
{
  mc_rtc::log::warning("[{}] deactivate not implemented in this planner", name());
}

bool ExternalFootstepPlanner::available() const
{
  return false;
}

void ExternalFootstepPlanner::requestPlan(const Request &)
{
  mc_rtc::log::warning("[{}] requestPlan not implemented in this planner", name());
}

bool ExternalFootstepPlanner::hasPlan() const noexcept
{
  return false;
}

Plan ExternalFootstepPlanner::popPlan()
{
  return Plan{};
}
} // namespace ExternalFootstepPlanner
} // namespace mc_plugin
