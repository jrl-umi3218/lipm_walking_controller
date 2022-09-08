#pragma once

#include <mc_rtc/gui/StateBuilder.h>

#include <ExternalFootstepPlanner/Plan.h>
#include <ExternalFootstepPlanner/Request.h>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

/**
 * @brief Base interface to interact with external planners
 */
struct ExternalFootstepPlanner
{
  virtual ~ExternalFootstepPlanner();
  virtual void configure(const mc_rtc::Configuration &){};
  virtual void activate();
  virtual void deactivate();
  virtual bool available() const;

  /**
   * @brief
   *
   * @param request Requested parameters for the plan (start, finish, time, etc)
   */
  virtual void requestPlan(const Request & request);
  virtual bool hasPlan() const noexcept;
  virtual Plan popPlan();
  virtual std::string name() const = 0;
  virtual void addToGUI(mc_rtc::gui::StateBuilder &){};
  virtual void removeFromGUI(mc_rtc::gui::StateBuilder &){};
};

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin
