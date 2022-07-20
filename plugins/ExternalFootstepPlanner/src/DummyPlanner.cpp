#include <mc_rtc/logging.h>

#include <ExternalFootstepPlanner/DummyPlanner.h>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

void DummyPlanner::requestPlan(const Request & /* request */)
{
  mc_rtc::log::warning("[{}] Requesting a plan to the Dummy Planner has no effect", name());
}

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin
