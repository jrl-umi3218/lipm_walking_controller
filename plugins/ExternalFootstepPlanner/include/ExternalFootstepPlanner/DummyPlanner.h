#pragma once

#include <ExternalFootstepPlanner/ExternalFootstepPlanner.h>
#include <ExternalFootstepPlanner/Plan.h>
#include <future>
#include <mutex>
#include <thread>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

/**
 * @brief Base interface to interact with external planners
 */
struct DummyPlanner : ExternalFootstepPlanner
{
  /**
   * @brief
   *
   * @param request Requested parameters for the plan (start, finish, time, etc)
   * @return std::future<boost::optional<Plan>> A future plan to be returned once processed
   */
  void requestPlan(const Request & request) override;

  inline std::string name() const override
  {
    return "DummyPlanner";
  }
};

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin
