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

#include "Initial.h"

namespace lipm_walking
{

void states::Initial::configure(const mc_rtc::Configuration & config)
{
  config("resetFloatingBaseToPlan", resetFloatingBaseToPlan_);
  config("updateContactFramesToCurrentSurface", updateContactFramesToCurrentSurface_);
  config("resetPosture", resetPosture_);
  config("resetPendulumHeight", resetPendulumHeight_);
}

void states::Initial::start()
{
  auto & ctl = controller();

  postureTaskIsActive_ = true;
  postureTaskWasActive_ = true;
  startStandingButton_ = false;
  startStanding_ = ctl.config()("autoplay", false);

  internalReset();

  logger().addLogEntry("walking_phase", []() { return -2.; });

  if(gui())
  {
    gui()->removeElement({"Walking", "Main"}, "Pause walking");
  }

  runState(); // don't wait till next cycle to update reference and tasks
}

void states::Initial::teardown()
{
  logger().removeLogEntry("walking_phase");

  if(gui())
  {
    hideStartStandingButton();
  }
}

void states::Initial::runState()
{
  auto & ctl = controller();
  if(resetPosture_)
  {
    postureTaskIsActive_ = (ctl.postureTask->speed().norm() > 1e-2);
    if(postureTaskIsActive_)
    {
      hideStartStandingButton();
      postureTaskWasActive_ = true;
    }
    else if(postureTaskWasActive_)
    {
      showStartStandingButton();
      postureTaskWasActive_ = false;
    }
  }
  else
  {
    showStartStandingButton();
    postureTaskIsActive_ = false;
    startStanding_ = true;
  }
}

bool states::Initial::checkTransitions()
{
  if(startStanding_ && !postureTaskIsActive_)
  {
    output("Standing");
    return true;
  }
  return false;
}

void states::Initial::showStartStandingButton()
{
  if(!startStandingButton_ && gui())
  {
    using namespace mc_rtc::gui;
    gui()->addElement({"Walking", "Main"}, Button("Start standing", [this]() { startStanding_ = true; }));
    startStandingButton_ = true;
  }
}

void states::Initial::hideStartStandingButton()
{
  if(startStandingButton_ && gui())
  {
    gui()->removeElement({"Walking", "Main"}, "Start standing");
    startStandingButton_ = false;
  }
}

void states::Initial::internalReset()
{
  auto & ctl = controller();
  // FIXME:
  // - resets does not respect the foot plan position when pausing walking (feet come back align
  // dragging on the floor)
  // - Do we still need the posture tricks?

  // (1) update floating-base transforms of both robot mbc's
  if(resetFloatingBaseToPlan_)
  {
    auto X_0_fb = ctl.supportContact().robotTransform(ctl.controlRobot());
    ctl.controlRobot().posW(X_0_fb);
    ctl.controlRobot().velW(sva::MotionVecd::Zero());
    ctl.realRobot().posW(X_0_fb);
    ctl.realRobot().velW(sva::MotionVecd::Zero());
  }

  // (2) update contact frames to coincide with surface ones
  ctl.loadFootstepPlan(ctl.plan.name);

  // (3) reset solver tasks
  if(resetPosture_)
  {
    ctl.postureTask->posture(ctl.halfSitPose);
  }

  // (4) reset controller attributes
  ctl.leftFootRatio(0.5);
  ctl.nbMPCFailures_ = 0;
  ctl.pauseWalking = false;
  ctl.pauseWalkingRequested = false;

  if(resetPendulumHeight_)
  {
    // XXX Default height is the same as that defined hidden in Stephane's
    // Pendulum::reset() function. This should probably be read from config
    // or use the robot height above ground instead.
    constexpr double DEFAULT_HEIGHT = 0.8; // [m]
    double lambda = mc_rtc::constants::GRAVITY / DEFAULT_HEIGHT;
    ctl.pendulum().reset(lambda, ctl.controlRobot().com());
  }

  ctl.stopLogSegment();
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("Initial", lipm_walking::states::Initial)
