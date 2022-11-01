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

#include "DoubleSupport.h"

namespace lipm_walking
{

using ContactState = mc_tasks::lipm_stabilizer::ContactState;

void states::DoubleSupport::handleExternalPlan()
{
  auto & ctl = controller();

  if(ctl.externalFootstepPlanner.hasPlan(ExternalPlanner::DoubleSupport))
  {
    ctl.plan.resetContacts(ctl.externalFootstepPlanner.plan());
    ctl.updatePlan("external");
  }
  else if(ctl.externalFootstepPlanner.planRequested(ExternalPlanner::DoubleSupport))
  {
    mc_rtc::log::warning("[{}] An external plan was requested for this DSP but is not available, pause walking",
                         name());
    ctl.externalFootstepPlanner.cancelRequest();
    stopDuringThisDSP_ = true;
    ctl.pauseWalking = true;
  }
}

void states::DoubleSupport::start()
{
  auto & ctl = controller();
  ctl.walkingState = WalkingState::DoubleSupport;

  double phaseDuration = ctl.doubleSupportDuration(); // careful! side effect here

  duration_ = phaseDuration;
  initLeftFootRatio_ = ctl.leftFootRatio();
  if(phaseDuration <= ctl.timeStep)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[{}] The double support phase duration cannot be "
                                                        "less than the controller's timestep (requested {} <= {})",
                                                        name(), duration_, ctl.timeStep);
  }
  remTime_ = duration_;
  stateTime_ = 0.;
  stopDuringThisDSP_ = ctl.pauseWalking;
  if(phaseDuration > ctl.timeStep)
  {
    timeSinceLastPreviewUpdate_ = 2 * PREVIEW_UPDATE_PERIOD; // update at transition...
  }
  else // ... unless DSP duration is zero
  {
    timeSinceLastPreviewUpdate_ = 0.;
  }

  if(ctl.plan.name == "external")
  {
    handleExternalPlan();
  }

  const std::string & targetSurfaceName = ctl.targetContact().surfaceName;
  auto actualTargetPose = ctl.controlRobot().surfacePose(targetSurfaceName);
  ctl.plan.goToNextFootstep(actualTargetPose);

  if(ctl.isLastDSP()) // called after goToNextFootstep
  {
    stopDuringThisDSP_ = true;
  }

  if(ctl.prevContact().surfaceName == "LeftFootCenter")
  {
    ctl.setContacts({{ContactState::Left, ctl.prevContact().pose}, {ContactState::Right, ctl.supportContact().pose}});
    targetLeftFootRatio_ = 0.;
  }
  else // (ctl.prevContact().surfaceName == "RightFootCenter")
  {
    ctl.setContacts({{ContactState::Left, ctl.supportContact().pose}, {ContactState::Right, ctl.prevContact().pose}});
    targetLeftFootRatio_ = 1.;
  }
  if(stopDuringThisDSP_)
  {
    targetLeftFootRatio_ = 0.5;
  }

  logger().addLogEntry("rem_phase_time", [this]() { return remTime_; });
  logger().addLogEntry("walking_phase", []() { return 2.; });

  runState(); // don't wait till next cycle to update reference and tasks
}

void states::DoubleSupport::teardown()
{
  logger().removeLogEntry("rem_phase_time");
  logger().removeLogEntry("walking_phase");
}

void states::DoubleSupport::runState()
{
  auto & ctl = controller();
  double dt = ctl.timeStep;

  if(remTime_ > 0 && timeSinceLastPreviewUpdate_ > PREVIEW_UPDATE_PERIOD
     && !(stopDuringThisDSP_ && remTime_ < PREVIEW_UPDATE_PERIOD))
  {
    updatePreview();
  }

  double x = clamp(remTime_ / duration_, 0., 1.);
  ctl.leftFootRatio(x * initLeftFootRatio_ + (1. - x) * targetLeftFootRatio_);

  ctl.preview->integrate(pendulum(), dt);
  pendulum().completeIPM(ctl.prevContact().p(), ctl.prevContact().normal());
  pendulum().resetCoMHeight(ctl.plan.comHeight(), ctl.prevContact().p(), ctl.prevContact().normal());
  controller().stabilizer()->target(pendulum().com(), pendulum().comd(), pendulum().comdd(), pendulum().zmp());

  remTime_ -= dt;
  stateTime_ += dt;
  timeSinceLastPreviewUpdate_ += dt;
}

bool states::DoubleSupport::checkTransitions()
{
  auto & ctl = controller();

  if(!stopDuringThisDSP_ && remTime_ < 0.)
  {
    output("SingleSupport");
    return true;
  }
  if(stopDuringThisDSP_ && remTime_ < -0.5)
  {
    if(!ctl.isLastDSP())
    {
      ctl.plan.restorePreviousFootstep(); // current one is for next SSP
    }
    output("Standing");
    return true;
  }
  return false;
}

void states::DoubleSupport::updatePreview()
{
  auto & ctl = controller();
  ctl.mpc().contacts(ctl.prevContact(), ctl.supportContact(), ctl.targetContact());
  if(stopDuringThisDSP_)
  {
    ctl.mpc().phaseDurations(0., remTime_, 0.);
  }
  else
  {
    ctl.mpc().phaseDurations(0., remTime_, ctl.singleSupportDuration());
  }
  if(ctl.updatePreview())
  {
    timeSinceLastPreviewUpdate_ = 0.;
  }
  else
  {
    mc_rtc::log::warning("No capture trajectory, resuming walking");
    stopDuringThisDSP_ = false;
  }
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("LIPMWalking::DoubleSupport", lipm_walking::states::DoubleSupport)
