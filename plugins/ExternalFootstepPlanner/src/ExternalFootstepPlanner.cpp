#include <ExternalFootstepPlanner/ExternalFootstepPlanner.h>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

ExternalFootstepPlanner::~ExternalFootstepPlanner()
{
  deactivate();
}

void ExternalFootstepPlanner::activate() {}
void ExternalFootstepPlanner::deactivate() {}

bool ExternalFootstepPlanner::available() const
{
  return false;
}

void ExternalFootstepPlanner::requestPlan(const Request &) {}

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
