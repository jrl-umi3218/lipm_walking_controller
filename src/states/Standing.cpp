/*
 * Copyright (c) 2018-2019, CNRS-UM LIRMM
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "Standing.h"

#include <lipm_walking/utils/clamp.h>
#include <lipm_walking/utils/world.h>

namespace lipm_walking
{

void states::Standing::start()
{
  auto & ctl = controller();

  planChanged_ = false;
  lastInterpolatorIter_ = ctl.planInterpolator.nbIter;
  startWalking_ = ctl.config()("autoplay", false);

  if(ctl.isLastDSP())
  {
    ctl.loadFootstepPlan(ctl.plan.name);
  }

  // stabilizer().contactState(ContactState::DoubleSupport);
  // stabilizer().setContact(stabilizer().leftFootTask, leftFootContact_);
  // stabilizer().setContact(stabilizer().rightFootTask, rightFootContact_);
  // stabilizer().addTasks(ctl.solver());
  // ctl.solver().addTask(ctl.pelvisTask);
  // ctl.solver().addTask(ctl.torsoTask);

  updatePlan(ctl.plan.name);

  logger().addLogEntry("walking_phase", []() { return 3.; });
  ctl.stopLogSegment();

  if(startWalking_) // autoplay
  {
    auto plans = ctl.config()("autoplay_plans", std::vector<std::string>{});
    if(plans.size() == 0)
    {
      startWalking_ = false;
      ctl.config().add("autoplay", false);
    }
    else
    {
      std::string plan = plans[0];
      plans.erase(plans.begin());
      ctl.config().add("autoplay_plans", plans);
      lastInterpolatorIter_++;
      updatePlan(plan);
    }
  }

  if(gui())
  {
    using namespace mc_rtc::gui;
    gui()->removeElement({"Walking", "Main"}, "Pause walking");
    gui()->addElement({"Walking", "Main"}, ComboInput("Footstep plan", ctl.planInterpolator.availablePlans(),
                                                      [&ctl]() { return ctl.plan.name; },
                                                      [this](const std::string & name) { updatePlan(name); }));
    gui()->addElement({"Walking", "Main"}, Button("Start walking", [this]() { startWalking(); }));
  }

  runState(); // don't wait till next cycle to update reference and tasks
}

void states::Standing::teardown()
{
  if(gui())
  {
    gui()->removeCategory({"Standing"});
    gui()->removeCategory({"Walking"});
  }
}

void states::Standing::runState()
{
  checkPlanUpdates();
}

bool states::Standing::checkTransitions()
{
  if(!startWalking_)
  {
    return false;
  }
  else
  {
    output("DoubleSupport");
    return true;
  }
}

void states::Standing::checkPlanUpdates()
{
  auto & ctl = controller();
  if(ctl.planInterpolator.nbIter > lastInterpolatorIter_)
  {
    ctl.loadFootstepPlan(ctl.planInterpolator.customPlanName());
    lastInterpolatorIter_ = ctl.planInterpolator.nbIter;
    planChanged_ = true;
  }
  if(planChanged_)
  {
    if(gui())
    {
      gui()->removeElement({"Walking", "Main"}, "Resume walking");
      gui()->removeElement({"Walking", "Main"}, "Start walking");
      gui()->addElement({"Walking", "Main"}, mc_rtc::gui::Button("Start walking", [this]() { startWalking(); }));
    }
    planChanged_ = false;
  }
}

void states::Standing::startWalking()
{
  auto & ctl = controller();
  if(ctl.isLastSSP())
  {
    LOG_ERROR("No footstep in contact plan");
    return;
  }
  startWalking_ = true;
  gui()->addElement({"Walking", "Main"},
                    mc_rtc::gui::Button("Pause walking", [&ctl]() { ctl.pauseWalkingCallback(/* verbose = */ true); }));
}

void states::Standing::updatePlan(const std::string & name)
{
  auto & ctl = controller();
  if(name.find("custom") != std::string::npos)
  {
    if(!ctl.planInterpolator.isShown)
    {
      ctl.planInterpolator.addGUIElements();
      ctl.planInterpolator.isShown = true;
    }
    if(name.find("backward") != std::string::npos)
    {
      ctl.planInterpolator.restoreBackwardTarget();
    }
    else if(name.find("forward") != std::string::npos)
    {
      ctl.planInterpolator.restoreForwardTarget();
    }
    else if(name.find("lateral") != std::string::npos)
    {
      ctl.planInterpolator.restoreLateralTarget();
    }
    ctl.loadFootstepPlan(ctl.planInterpolator.customPlanName());
  }
  else // new plan is not custom
  {
    if(ctl.planInterpolator.isShown)
    {
      ctl.planInterpolator.removeGUIElements();
      ctl.planInterpolator.isShown = false;
    }
    ctl.loadFootstepPlan(name);
  }
  planChanged_ = true;
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("StartWalkingChoice", lipm_walking::states::Standing)
