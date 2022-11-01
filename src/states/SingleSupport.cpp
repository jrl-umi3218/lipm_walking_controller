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

#include "SingleSupport.h"

namespace lipm_walking
{

void states::SingleSupport::handleExternalPlan()
{
  using Foot = mc_plugin::ExternalFootstepPlanner::Foot;
  auto & ctl = controller();

  double allowedTime = ctl.externalFootstepPlanner.allowedTimeSingleSupport();
  if(allowedTime >= duration_)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Maximum allowed time ({:.3f}) for the external planner "
                                                     "cannot be greater than the single support duration ({:.3f})",
                                                     name(), allowedTime, duration_);
  }

  if(ctl.externalFootstepPlanner.planningRequested())
  {
    if(remTime_ > allowedTime)
    { // If we have enough time left to compute a plan,
      // then we request a plan to be used during the next DoubleSupport phase
      if(ctl.supportContact().surfaceName == "LeftFootCenter")
      {
        const auto lf_start = utils::SE2d{ctl.supportContact().pose};
        const auto rf_start = utils::SE2d{ctl.targetContact().pose};
        Foot supportFoot = Foot::Right;
        ctl.externalFootstepPlanner.requestPlan(ExternalPlanner::DoubleSupport, supportFoot, lf_start, rf_start,
                                                allowedTime);
      }
      else
      {
        const auto rf_start = utils::SE2d{ctl.supportContact().pose};
        const auto lf_start = utils::SE2d{ctl.targetContact().pose};
        Foot supportFoot = Foot::Left;
        ctl.externalFootstepPlanner.requestPlan(ExternalPlanner::DoubleSupport, supportFoot, lf_start, rf_start,
                                                allowedTime);
      }
    }
  }
}

void states::SingleSupport::start()
{
  auto & ctl = controller();
  ctl.walkingState = WalkingState::SingleSupport;
  auto & supportContact = ctl.supportContact();
  auto & targetContact = ctl.targetContact();

  duration_ = ctl.singleSupportDuration();
  hasUpdatedMPCOnce_ = false;
  remTime_ = ctl.singleSupportDuration();
  stateTime_ = 0.;
  timeSinceLastPreviewUpdate_ = 0.; // don't update at transition

  if(supportContact.surfaceName == "LeftFootCenter")
  {
    ctl.leftFootRatio(1.);
    ctl.setContacts({{ContactState::Left, supportContact.pose}});
    swingFootTask = ctl.swingFootTaskRight_;
  }
  else
  {
    ctl.leftFootRatio(0.);
    ctl.setContacts({{ContactState::Right, supportContact.pose}});
    swingFootTask = ctl.swingFootTaskLeft_;
  }
  swingFootTask->reset();
  ctl.solver().addTask(swingFootTask);

  swingFoot_.landingDuration(ctl.plan.landingDuration());
  swingFoot_.landingPitch(ctl.plan.landingPitch());
  swingFoot_.takeoffDuration(ctl.plan.takeoffDuration());
  swingFoot_.takeoffOffset(ctl.plan.takeoffOffset());
  swingFoot_.takeoffPitch(ctl.plan.takeoffPitch());
  swingFoot_.reset(swingFootTask->surfacePose(), targetContact.pose, duration_, ctl.plan.swingHeight());

  logger().addLogEntry("rem_phase_time", [this]() { return remTime_; });
  logger().addLogEntry("walking_phase", []() { return 1.; });
  swingFoot_.addLogEntries(logger());

  runState(); // don't wait till next cycle to update reference and tasks
}

void states::SingleSupport::teardown()
{
  controller().solver().removeTask(swingFootTask);

  logger().removeLogEntry("contact_impulse");
  logger().removeLogEntry("rem_phase_time");
  logger().removeLogEntry("walking_phase");
  swingFoot_.removeLogEntries(logger());
}

bool states::SingleSupport::checkTransitions()
{
  if(remTime_ < 0.)
  {
    output("DoubleSupport");
    return true;
  }
  return false;
}

void states::SingleSupport::runState()
{
  auto & ctl = controller();
  double dt = ctl.timeStep;

  if(ctl.plan.name == "external")
  {
    handleExternalPlan();
  }

  updateSwingFoot();
  if(timeSinceLastPreviewUpdate_ > PREVIEW_UPDATE_PERIOD)
  {
    updatePreview();
  }

  ctl.preview->integrate(pendulum(), dt);
  if(hasUpdatedMPCOnce_)
  {
    pendulum().resetCoMHeight(ctl.plan.comHeight(), ctl.supportContact().p(), ctl.supportContact().normal());
    pendulum().completeIPM(ctl.supportContact().p(), ctl.supportContact().normal());
  }
  else // still in DSP of preview
  {
    pendulum().completeIPM(ctl.prevContact().p(), ctl.prevContact().normal());
  }

  stabilizer()->target(pendulum().com(), pendulum().comd(), pendulum().comdd(), pendulum().zmp());

  remTime_ -= dt;
  stateTime_ += dt;
  timeSinceLastPreviewUpdate_ += dt;
}

void states::SingleSupport::updateSwingFoot()
{
  auto & ctl = controller();
  auto & targetContact = ctl.targetContact();
  auto & supportContact = ctl.supportContact();
  double dt = ctl.timeStep;

  if(!stabilizer()->inDoubleSupport())
  {
    bool liftPhase = (remTime_ > duration_ / 3.);
    bool touchdownDetected = detectTouchdown(swingFootTask, targetContact.pose);
    if(liftPhase || !touchdownDetected)
    {
      swingFoot_.integrate(dt);
      swingFootTask->target(swingFoot_.pose());
      // T_0_s transforms a MotionVecd variable from world to surface frame
      sva::PTransformd T_0_s(swingFootTask->surfacePose().rotation());
      swingFootTask->refVelB(T_0_s * swingFoot_.vel());
      swingFootTask->refAccel(T_0_s * swingFoot_.accel());
    }
    else
    {
      if(supportContact.surfaceName == "LeftFootCenter")
      {
        stabilizer()->setContacts(
            {{ContactState::Left, supportContact.pose}, {ContactState::Right, targetContact.pose}});
      }
      else
      {
        stabilizer()->setContacts(
            {{ContactState::Left, targetContact.pose}, {ContactState::Right, supportContact.pose}});
      }
    }
  }
}

bool states::SingleSupport::detectTouchdown(const std::shared_ptr<mc_tasks::SurfaceTransformTask> footTask,
                                            const sva::PTransformd & contactPose)
{
  const sva::PTransformd X_0_s = footTask->surfacePose();
  const sva::PTransformd & X_0_c = contactPose;
  sva::PTransformd X_c_s = X_0_s * X_0_c.inv();
  double xDist = std::abs(X_c_s.translation().x());
  double yDist = std::abs(X_c_s.translation().y());
  double zDist = std::abs(X_c_s.translation().z());
  double Fz = controller().robot().surfaceWrench(footTask->surface()).force().z();
  return (xDist < 0.03 && yDist < 0.03 && zDist < 0.03 && Fz > 50.);
}

void states::SingleSupport::updatePreview()
{
  auto & ctl = controller();
  ctl.mpc().contacts(ctl.supportContact(), ctl.targetContact(), ctl.nextContact());
  if(ctl.isLastSSP() || ctl.pauseWalking)
  {
    ctl.nextDoubleSupportDuration(ctl.plan.finalDSPDuration());
    ctl.mpc().phaseDurations(remTime_, ctl.plan.finalDSPDuration(), 0.);
  }
  else
  {
    ctl.mpc().phaseDurations(remTime_, ctl.doubleSupportDuration(), ctl.singleSupportDuration());
  }
  if(ctl.updatePreview())
  {
    timeSinceLastPreviewUpdate_ = 0.;
    hasUpdatedMPCOnce_ = true;
  }
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("LIPMWalking::SingleSupport", lipm_walking::states::SingleSupport)
