#pragma once

#include <mc_control/MCController.h>

#include <ExternalFootstepPlanner/ExternalFootstepPlanner.h>
#include <ExternalFootstepPlanner/Plan.h>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

/**
 * @brief Base interface to interact with external planners
 */
struct HybridPlanner : ExternalFootstepPlanner
{
  HybridPlanner(mc_control::MCController & ctl);
  ~HybridPlanner();

  void configure(const mc_rtc::Configuration & config) override;
  void setVelocity(const Eigen::Vector3d & velocity);

  /**
   * @brief
   *
   * @param request Requested parameters for the plan (start, finish, time, etc)
   * @return std::future<boost::optional<Plan>> A future plan to be returned once processed
   */
  void requestPlan(const Request & request) override;
  bool hasPlan() const noexcept override;
  Plan popPlan() override;

  std::string name() const override
  {
    return "HybridPlanner";
  }

  bool available() const override
  {
    // return ctl_.datastore().has("footstep_planner::compute_plan");
    return true;
  }

  void activate() override;
  void deactivate() override;

protected:
  void rosThread();

protected:
  mc_control::MCController & ctl_;
  std::string supportFootName_ = "LeftFoot";
  bool computing_ = false;
  double Tp_ = 0;
  double delta_ = 0;
  Plan plan_;
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
};

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin
