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

#include <mc_rtc/constants.h>

#include <ExternalFootstepPlanner/Plan.h>
#include <ExternalFootstepPlanner/Request.h>
#include <lipm_walking/utils/clamp.h>

namespace lipm_walking
{

namespace
{
constexpr double COM_STIFFNESS = 5.; // standing has CoM set-point task
}

void states::Standing::configure(const mc_rtc::Configuration & config)
{
  config("autoplay", autoplay_);
  config("autoplay_plans", autoplay_plans_);
}

void states::Standing::start()
{
  auto & ctl = controller();
  ctl.startWalking = autoplay_;
  ctl.walkingState = WalkingState::Standing;
  // Reset pendulum state starting from the current CoM state
  // This is done to ensure that there is no discontinuity when entering the
  // Standing state from any external state.
  double lambda = mc_rtc::constants::GRAVITY / ctl.robot().com().z();
  ctl.pendulum().reset(lambda, ctl.robot().com(), ctl.robot().comVelocity(), ctl.robot().comAcceleration());

  supportContact_ = ctl.supportContact();
  targetContact_ = ctl.targetContact();

  leftFootRatio_ = ctl.leftFootRatio();
  ctl.isWalking = false;
  if(supportContact_.surfaceName == "RightFootCenter")
  {
    leftFootContact_ = targetContact_;
    rightFootContact_ = supportContact_;
  }
  else if(supportContact_.surfaceName == "LeftFootCenter")
  {
    leftFootContact_ = supportContact_;
    rightFootContact_ = targetContact_;
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("Unknown surface name: {}", supportContact_.surfaceName);
  }

  if(ctl.isLastDSP())
  {
    // When reaching the end of an external plan, do not repeat the previous plan
    // This prevents walking until another plan is requested and received.
    if(ctl.plan.name == "external")
    {
      ctl.plan.resetContacts(ctl.planInterpolator.getPlan("external").contacts());
    }
    ctl.loadFootstepPlan(ctl.plan.name);
  }

  ctl.setContacts({{ContactState::Left, leftFootContact_.pose}, {ContactState::Right, rightFootContact_.pose}});

  if(!ctl.pauseWalking)
  {
    ctl.updatePlan(ctl.plan.name);
  }
  else
  {
    mc_rtc::log::info("[Standing] Walking is paused");
  }
  updateTarget(leftFootRatio_);

  logger().addLogEntry("walking_phase", []() { return 3.; });
  ctl.stopLogSegment();

  if(ctl.startWalking && !ctl.pauseWalking) // autoplay
  {
    auto plans = std::vector<std::string>{};
    if(autoplay_plans_.size())
    { // If defined, use plans from local state configuration
      plans = autoplay_plans_;
    }
    else
    { // otherwise use global plans
      plans = ctl.config()("autoplay_plans", std::vector<std::string>{});
    }
    if(plans.size() == 0)
    {
      ctl.startWalking = false;
      ctl.config().add("autoplay", false);
    }
    else
    {
      std::string plan = plans[0];
      plans.erase(plans.begin());
      ctl.config().add("autoplay_plans", plans);
      ctl.updatePlan(plan);
    }
  }

  if(gui())
  {
    using namespace mc_rtc::gui;
    auto & gui_ = *gui();
    gui_.removeElement({"Walking", "Main"}, "Pause walking");
    gui_.removeElement({"Walking", "Main"}, "Resume walking");
    gui_.removeElement({"Walking", "Main"}, "Start walking");
    gui_.addElement({"Walking", "Main"},
                    ComboInput(
                        "Footstep plan", ctl.planInterpolator.availablePlans(), [&ctl]() { return ctl.plan.name; },
                        [&ctl](const std::string & name) { ctl.updatePlan(name); }));
    gui_.addElement({"Standing"},
                    NumberInput(
                        "CoM target [0-1]", [this]() { return std::round(leftFootRatio_ * 10.) / 10.; },
                        [this](double leftFootRatio) { updateTarget(leftFootRatio); }),
                    Label("Left foot force [N]",
                          [&ctl]() { return ctl.realRobot().forceSensor("LeftFootForceSensor").force().z(); }),
                    Label("Right foot force [N]",
                          [&ctl]() { return ctl.realRobot().forceSensor("RightFootForceSensor").force().z(); }),
                    Button("Go to left foot", [this]() { updateTarget(1.); }),
                    Button("Go to middle", [this]() { updateTarget(0.5); }),
                    Button("Go to right foot", [this]() { updateTarget(0.); }));
    gui_.addElement({"Walking", "Main"},
                    Button(!controller().pauseWalking || (supportContact_.id == 0) ? "Start walking" : "Resume walking",
                           [this]() { startWalking(); }));
  }

  runState(); // don't wait till next cycle to update reference and tasks
}

void states::Standing::teardown()
{
  logger().removeLogEntry("walking_phase");

  if(gui())
  {
    gui()->removeCategory({"Standing"});
    gui()->removeElement({"Walking", "Main"}, "Footstep plan");
    gui()->removeElement({"Walking", "Main"}, "Gait");
    gui()->removeElement({"Walking", "Main"}, "Go to middle");
    gui()->removeElement({"Walking", "Main"}, "Resume walking");
    gui()->removeElement({"Walking", "Main"}, "Start walking");
  }
}

void states::Standing::runState()
{
  checkPlanUpdates();

  auto & ctl = controller();
  auto & pendulum = ctl.pendulum();

  Eigen::Vector3d comTarget = copTarget_ + Eigen::Vector3d{0., 0., ctl.plan.comHeight()};
  const Eigen::Vector3d & com_i = pendulum.com();
  const Eigen::Vector3d & comd_i = pendulum.comd();
  const Eigen::Vector3d & cop_f = copTarget_;

  double K = COM_STIFFNESS;
  double D = 2 * std::sqrt(K);
  Eigen::Vector3d comdd = K * (comTarget - com_i) - D * comd_i;
  Eigen::Vector3d n = supportContact_.normal();
  double lambda = n.dot(comdd + mc_rtc::constants::gravity) / n.dot(com_i - cop_f);
  Eigen::Vector3d zmp = com_i - (mc_rtc::constants::gravity + comdd) / lambda;

  pendulum.integrateIPM(zmp, lambda, ctl.timeStep);
  ctl.leftFootRatio(leftFootRatio_);
  ctl.stabilizer()->target(pendulum.com(), pendulum.comd(), pendulum.comdd(), pendulum.zmp());
}

void states::Standing::handleExternalPlan()
{
  using Foot = mc_plugin::ExternalFootstepPlanner::Foot;
  auto & ctl = controller();

  if(ctl.externalFootstepPlanner.planningRequested())
  {
    // Request a plan that should be received by the standing state
    // e.g this means we will wait for the plan here before completing
    const auto lf_start = utils::SE2d{controller().robot().surfacePose("LeftFootCenter")};
    const auto rf_start = utils::SE2d{controller().robot().surfacePose("RightFootCenter")};

    // XXX always starts with Left support foot
    // Should probably record the last used support foot when entering standing state and start from the other foot
    // instead to resume walk more naturally
    auto defaultFoot = Foot::Right;
    if(supportContact_.surfaceName == "RightFootCenter")
    {
      defaultFoot = Foot::Left;
    }
    auto lf = controller().robot().surfacePose("LeftFootCenter") * controller().robot().posW().inv();
    auto rf = controller().robot().surfacePose("RightFootCenter") * controller().robot().posW().inv();
    Eigen::Vector3d ref = ctl.datastore().call<Eigen::Vector3d>("HybridPlanner::GetVelocity");
    if(ref.x() > 0.02)
    {
      bool rf_front_lf = rf.translation().x() > lf.translation().x() + 0.02;
      bool lf_front_rf = lf.translation().x() > rf.translation().x() + 0.02;
      if(rf_front_lf)
      {
        defaultFoot = Foot::Right;
      }
      if(lf_front_rf)
      {
        defaultFoot = Foot::Left;
      }
    }
    else if(ref.x() < -0.02)
    {
      bool rf_front_lf = rf.translation().x() > lf.translation().x() + 0.02;
      bool lf_front_rf = lf.translation().x() > rf.translation().x() + 0.02;
      if(rf_front_lf)
      {
        defaultFoot = Foot::Left;
      }
      if(lf_front_rf)
      {
        defaultFoot = Foot::Right;
      }
    }
    ctl.externalFootstepPlanner.requestPlan(
        ExternalPlanner::Standing, defaultFoot, lf_start, rf_start,
        ctl.externalFootstepPlanner.allowedTimeStanding()); // XXX hardcoded allowed time
  }

  if(ctl.externalFootstepPlanner.hasPlan(ExternalPlanner::Standing)
     || ctl.externalFootstepPlanner.hasPlan(ExternalPlanner::DoubleSupport))
  {
    // If we have received a plan requested during this Standing phase or the previous DoubleSupport phase
    ctl.plan.resetContacts(ctl.externalFootstepPlanner.plan());
    ctl.updatePlan("external");
  }
}

void states::Standing::checkPlanUpdates()
{
  auto & ctl = controller();

  if(ctl.plan.name == "external")
  {
    handleExternalPlan();
  }
  else if(ctl.planInterpolator.checkPlanUpdated())
  {
    ctl.loadFootstepPlan(ctl.planInterpolator.customPlanName());
  }
}

void states::Standing::updateTarget(double leftFootRatio)
{
  auto & sole = controller().sole();
  if(!controller().stabilizer()->inDoubleSupport())
  {
    mc_rtc::log::error("Cannot update CoM target while in single support");
    return;
  }
  leftFootRatio = clamp(leftFootRatio, 0., 1., "Standing target");
  sva::PTransformd X_0_lfr =
      sva::interpolate(rightFootContact_.anklePose(sole), leftFootContact_.anklePose(sole), leftFootRatio);
  copTarget_ = X_0_lfr.translation();
  leftFootRatio_ = leftFootRatio;
}

bool states::Standing::checkTransitions()
{
  auto & ctl = controller();

  if(!ctl.startWalking || ctl.pauseWalking)
  {
    return false;
  }

  ctl.mpc().contacts(supportContact_, targetContact_, ctl.nextContact());
  ctl.mpc().phaseDurations(0., ctl.plan.initDSPDuration(), ctl.singleSupportDuration());
  if(ctl.updatePreview())
  {
    ctl.nextDoubleSupportDuration(ctl.plan.initDSPDuration());
    ctl.startLogSegment(ctl.plan.name);
    ctl.isWalking = true;
    output("DoubleSupport");
    return true;
  }
  return false;
}

void states::Standing::startWalking()
{
  auto & ctl = controller();
  if(ctl.isLastSSP())
  {
    mc_rtc::log::error("No footstep in contact plan");
    return;
  }
  ctl.startWalking = true;
  if(ctl.pauseWalking)
  {
    gui()->removeElement({"Walking", "Main"}, "Resume walking");
    ctl.pauseWalking = false;
  }
  gui()->addElement({"Walking", "Main"}, mc_rtc::gui::Button("Pause walking", [&ctl]() {
                      ctl.pauseWalkingCallback(/* verbose = */ false);
                    }));
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("LIPMWalking::Standing", lipm_walking::states::Standing)
