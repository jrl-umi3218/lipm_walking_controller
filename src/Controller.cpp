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

#include <mc_rbdyn/constants.h>
#include <mc_rbdyn/rpy_utils.h>

#include <lipm_walking/Controller.h>
#include <lipm_walking/utils/clamp.h>

using Color = mc_rtc::gui::Color;

namespace lipm_walking
{

Controller::Controller(std::shared_ptr<mc_rbdyn::RobotModule> robotModule,
                       double dt,
                       const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(robotModule, dt, config), planInterpolator(gui()), halfSitPose(controlRobot().mbc().q)
{
  auto robotConfig = config("robot_models")(controlRobot().name());
  auto planConfig = config("plans")(controlRobot().name());

  // Patch CoM height and step width in all plans
  std::vector<std::string> plans = planConfig.keys();
  double comHeight = robotConfig("com")("height");
  for(const auto & p : plans)
  {
    auto plan = planConfig(p);
    if(!plan.has("com_height"))
    {
      plan.add("com_height", comHeight);
    }
  }

  double postureStiffness = config("tasks")("posture")("stiffness");
  double postureWeight = config("tasks")("posture")("weight");
  postureTask = getPostureTask(robot().name());
  postureTask->stiffness(postureStiffness);
  postureTask->weight(postureWeight);

  // Set half-sitting pose for posture task
  const auto & halfSit = robotModule->stance();
  const auto & refJointOrder = robot().refJointOrder();
  for(unsigned i = 0; i < refJointOrder.size(); ++i)
  {
    if(robot().hasJoint(refJointOrder[i]))
    {
      halfSitPose[robot().jointIndexByName(refJointOrder[i])] = halfSit.at(refJointOrder[i]);
    }
  }

  // Read sole properties from robot model and configuration file
  sva::PTransformd X_0_lfc = controlRobot().surfacePose("LeftFootCenter");
  sva::PTransformd X_0_rfc = controlRobot().surfacePose("RightFootCenter");
  sva::PTransformd X_0_lf = controlRobot().surfacePose("LeftFoot");
  sva::PTransformd X_lfc_lf = X_0_lf * X_0_lfc.inv();
  sva::PTransformd X_rfc_lfc = X_0_lfc * X_0_rfc.inv();
  double stepWidth = X_rfc_lfc.translation().y();
  sole_ = robotConfig("sole");
  sole_.leftAnkleOffset = X_lfc_lf.translation().head<2>();

  // Configure MPC solver
  // TODO sole for the MPC
  mpcConfig_ = config("mpc");
  mpc_.sole(sole_);

  // Read footstep plans from configuration
  planInterpolator.configure(planConfig);
  planInterpolator.stepWidth(stepWidth);
  std::string initialPlan = planInterpolator.availablePlans()[0];
  config("initial_plan", initialPlan);
  loadFootstepPlan(initialPlan);

  // ====================
  // Create Stabilizer
  // - Default configuration from the robot module
  // - Additional configuration from the configuration, in section
  //    robot_name
  //    {
  //      Stabilizer
  //      {
  //        ... stabilizer configuration ...
  //      }
  //    }
  // ====================
  LOG_INFO("Loading default stabilizer configuration");
  auto stabiConf = robot().module().defaultLIPMStabilizerConfiguration();
  stabilizer_.reset(new mc_tasks::lipm_stabilizer::StabilizerTask(
      solver().robots(), solver().realRobots(), robot().robotIndex(), stabiConf.leftFootSurface,
      stabiConf.rightFootSurface, stabiConf.torsoBodyName, solver().dt()));
  if(config.has(robot().name()))
  {
    if(config.has("Stabilizer"))
    {
      LOG_INFO("Loading additional stabilizer configuration");
      stabiConf.load(config("Stabilizer"));
    }
  }
  stabilizer_->configure(stabiConf);

  // =========================
  // Create Swing foot tasks
  // =========================
  swingFootTaskRight_.reset(
      new mc_tasks::SurfaceTransformTask("RightFootCenter", robots(), robots().robotIndex(), 2000, 500));
  swingFootTaskLeft_.reset(
      new mc_tasks::SurfaceTransformTask("LeftFootCenter", robots(), robots().robotIndex(), 2000, 500));

  addLogEntries(logger());
  mpc_.addLogEntries(logger());

  if(gui_)
  {
    addGUIElements(gui_);
    mpc_.addGUIElements(gui_);
  }
  LOG_SUCCESS("LIPMWalking controller init done " << this)
}

void Controller::addLogEntries(mc_rtc::Logger & logger)
{
  logger.addLogEntry("controlRobot_LeftFoot", [this]() { return controlRobot().surfacePose("LeftFoot"); });
  logger.addLogEntry("controlRobot_LeftFootCenter", [this]() { return controlRobot().surfacePose("LeftFootCenter"); });
  logger.addLogEntry("controlRobot_RightFoot", [this]() { return controlRobot().surfacePose("RightFoot"); });
  logger.addLogEntry("controlRobot_RightFootCenter",
                     [this]() { return controlRobot().surfacePose("RightFootCenter"); });
  logger.addLogEntry("mpc_failures", [this]() { return nbMPCFailures_; });
  logger.addLogEntry("left_foot_ratio", [this]() { return leftFootRatio_; });
  logger.addLogEntry("left_foot_ratio_measured", [this]() { return measuredLeftFootRatio(); });
  logger.addLogEntry("plan_com_height", [this]() { return plan.comHeight(); });
  logger.addLogEntry("plan_double_support_duration", [this]() { return plan.doubleSupportDuration(); });
  logger.addLogEntry("plan_final_dsp_duration", [this]() { return plan.finalDSPDuration(); });
  logger.addLogEntry("plan_init_dsp_duration", [this]() { return plan.initDSPDuration(); });
  logger.addLogEntry("plan_landing_duration", [this]() { return plan.landingDuration(); });
  logger.addLogEntry("plan_landing_pitch", [this]() { return plan.landingPitch(); });
  logger.addLogEntry("plan_ref_vel", [this]() { return plan.supportContact().refVel; });
  logger.addLogEntry("plan_single_support_duration", [this]() { return plan.singleSupportDuration(); });
  logger.addLogEntry("plan_swing_height", [this]() { return plan.swingHeight(); });
  logger.addLogEntry("plan_takeoff_duration", [this]() { return plan.takeoffDuration(); });
  logger.addLogEntry("plan_takeoff_offset", [this]() { return plan.takeoffOffset(); });
  logger.addLogEntry("plan_takeoff_pitch", [this]() { return plan.takeoffPitch(); });
  logger.addLogEntry("realRobot_LeftFoot", [this]() { return realRobot().surfacePose("LeftFoot"); });
  logger.addLogEntry("realRobot_LeftFootCenter", [this]() { return realRobot().surfacePose("LeftFootCenter"); });
  logger.addLogEntry("realRobot_RightFoot", [this]() { return realRobot().surfacePose("RightFoot"); });
  logger.addLogEntry("realRobot_RightFootCenter", [this]() { return realRobot().surfacePose("RightFootCenter"); });
  logger.addLogEntry("realRobot_posW", [this]() { return realRobot().posW(); });
}

void Controller::addGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
{
  using namespace mc_rtc::gui;

  auto footStepPolygon = [](const Contact & contact) {
    std::vector<Eigen::Vector3d> polygon;
    polygon.push_back(contact.vertex0());
    polygon.push_back(contact.vertex1());
    polygon.push_back(contact.vertex2());
    polygon.push_back(contact.vertex3());
    return polygon;
  };

  gui->addElement(
      {"Markers", "Footsteps"},
      Polygon("TargetContact", Color::Red, [this, footStepPolygon]() { return footStepPolygon(targetContact()); }),
      Polygon("FootstepPlan", Color::Blue, [this, footStepPolygon]() {
        std::vector<std::vector<Eigen::Vector3d>> polygons;
        const auto & contacts = plan.contacts();
        for(unsigned i = 0; i < contacts.size(); i++)
        {
          auto & contact = contacts[i];
          double supportDist = (contact.p() - supportContact().p()).norm();
          double targetDist = (contact.p() - targetContact().p()).norm();
          constexpr double SAME_CONTACT_DIST = 0.005;
          // only display contact if it is not the support or target contact
          if(supportDist > SAME_CONTACT_DIST && targetDist > SAME_CONTACT_DIST)
          {
            polygons.push_back(footStepPolygon(contact));
          }
        }
        return polygons;
      }));

  gui->addElement({"Walking", "Main"},
                  Button("# EMERGENCY STOP",
                         [this]() {
                           LOG_ERROR("EMERGENCY STOP!");
                           emergencyStop = true;
                           this->interrupt();
                         }),
                  Button("Reset", [this]() {
                    LOG_WARNING("Reset to Initial state");
                    this->resume("Initial");
                  }));

  gui->addElement({"Walking", "CoM"}, Label("Plan name", [this]() { return plan.name; }));

  gui->addElement({"Walking", "Swing"}, Label("Plan name", [this]() { return plan.name; }),
                  NumberInput("Swing height [m]", [this]() { return plan.swingHeight(); },
                              [this](double height) { plan.swingHeight(height); }),
                  NumberInput("Takeoff duration [s]", [this]() { return plan.takeoffDuration(); },
                              [this](double duration) { plan.takeoffDuration(duration); }),
                  NumberInput("Takeoff pitch [rad]", [this]() { return plan.takeoffPitch(); },
                              [this](double pitch) { plan.takeoffPitch(pitch); }),
                  NumberInput("Landing duration [s]", [this]() { return plan.landingDuration(); },
                              [this](double duration) { plan.landingDuration(duration); }),
                  NumberInput("Landing pitch [rad]", [this]() { return plan.landingPitch(); },
                              [this](double pitch) { plan.landingPitch(pitch); }));

  gui->addElement({"Walking", "Timings"}, Label("Plan name", [this]() { return plan.name; }),
                  NumberInput("Initial DSP duration [s]", [this]() { return plan.initDSPDuration(); },
                              [this](double duration) { plan.initDSPDuration(duration); }),
                  NumberInput("SSP duration [s]", [this]() { return plan.singleSupportDuration(); },
                              [this](double duration) {
                                constexpr double T = ModelPredictiveControl::SAMPLING_PERIOD;
                                duration = std::round(duration / T) * T;
                                plan.singleSupportDuration(duration);
                              }),
                  NumberInput("DSP duration [s]", [this]() { return plan.doubleSupportDuration(); },
                              [this](double duration) {
                                constexpr double T = ModelPredictiveControl::SAMPLING_PERIOD;
                                duration = std::round(duration / T) * T;
                                plan.doubleSupportDuration(duration);
                              }),
                  NumberInput("Final DSP duration [s]", [this]() { return plan.finalDSPDuration(); },
                              [this](double duration) { plan.finalDSPDuration(duration); }));
}

void Controller::reset(const mc_control::ControllerResetData & data)
{
  mc_control::fsm::Controller::reset(data);

  stabilizer_->reset();

  if(gui_)
  {
    gui_->removeCategory({"Contacts"});
  }
}

void Controller::internalReset()
{
  // FIXME:
  // - resets does not respect the foot plan position when pausing walking (feet come back align
  // dragging on the floor)
  // - Do we still need the posture tricks?

  // (1) update floating-base transforms of both robot mbc's
  auto X_0_fb = supportContact().robotTransform(controlRobot());
  controlRobot().posW(X_0_fb);
  controlRobot().velW(sva::MotionVecd::Zero());
  realRobot().posW(X_0_fb);
  realRobot().velW(sva::MotionVecd::Zero());

  // (2) update contact frames to coincide with surface ones
  loadFootstepPlan(plan.name);

  // (3) reset solver tasks
  postureTask->posture(halfSitPose);

  // (4) reset controller attributes
  leftFootRatioJumped_ = true;
  leftFootRatio_ = 0.5;
  nbMPCFailures_ = 0;
  pauseWalking = false;
  pauseWalkingRequested = false;

  // XXX Default height is the same as that defined hidden in Stephan's
  // Pendulum::reset() function. This should probably be ready from config
  // or use the robot height above ground instead.
  constexpr double DEFAULT_HEIGHT = 0.8; // [m]
  double lambda = mc_rbdyn::constants::GRAVITY / DEFAULT_HEIGHT;
  pendulum_.reset(lambda, controlRobot().com());

  stopLogSegment();
}

void Controller::leftFootRatio(double ratio)
{
  double maxRatioVar = 1.5 * timeStep / plan.doubleSupportDuration();
  if(std::abs(ratio - leftFootRatio_) > maxRatioVar)
  {
    LOG_WARNING("Left foot ratio jumped from " << leftFootRatio_ << " to " << ratio);
    leftFootRatioJumped_ = true;
  }
  leftFootRatio_ = clamp(ratio, 0., 1., "leftFootRatio");
}

bool Controller::run()
{
  // Update observers
  anchorFrame(
      sva::interpolate(robot().surfacePose("RightFootCenter"), robot().surfacePose("LeftFootCenter"), leftFootRatio_));
  anchorFrameReal(sva::interpolate(realRobot().surfacePose("RightFootCenter"),
                                   realRobot().surfacePose("LeftFootCenter"), leftFootRatio_));

  if(emergencyStop)
  {
    return false;
  }
  if(pauseWalkingRequested)
  {
    pauseWalkingCallback();
  }
  if(!mc_control::fsm::Controller::running())
  {
    return mc_control::fsm::Controller::run();
  }

  ctlTime_ += timeStep;

  warnIfRobotIsInTheAir();

  bool ret = mc_control::fsm::Controller::run();
  if(mc_control::fsm::Controller::running())
  {
    postureTask->posture(halfSitPose); // reset posture in case the FSM updated it
  }
  return ret;
}

void Controller::pauseWalkingCallback(bool verbose)
{
  constexpr double MAX_HEIGHT_DIFF = 0.02; // [m]
  if(pauseWalking)
  {
    LOG_WARNING("Already pausing, how did you get there?");
    return;
  }
  else if(std::abs(supportContact().z() - targetContact().z()) > MAX_HEIGHT_DIFF)
  {
    if(!pauseWalkingRequested || verbose)
    {
      LOG_WARNING("Cannot pause on uneven ground, will pause later");
    }
    gui()->removeElement({"Walking", "Main"}, "Pause walking");
    pauseWalkingRequested = true;
  }
  else if(pauseWalkingRequested)
  {
    LOG_WARNING("Pausing now that contacts are at same level");
    pauseWalkingRequested = false;
    pauseWalking = true;
  }
  else // (!pauseWalkingRequested)
  {
    gui()->removeElement({"Walking", "Main"}, "Pause walking");
    pauseWalking = true;
  }
}

void Controller::warnIfRobotIsInTheAir()
{
  static bool isInTheAir = false;
  constexpr double CONTACT_THRESHOLD = 30.; // [N]
  double leftFootForce = realRobot().forceSensor("LeftFootForceSensor").force().z();
  double rightFootForce = realRobot().forceSensor("RightFootForceSensor").force().z();
  if(leftFootForce < CONTACT_THRESHOLD && rightFootForce < CONTACT_THRESHOLD)
  {
    if(!isInTheAir)
    {
      LOG_WARNING("Robot is in the air");
      isInTheAir = true;
    }
  }
  else
  {
    if(isInTheAir)
    {
      LOG_INFO("Robot is on the ground again");
      isInTheAir = false;
    }
  }
}

void Controller::loadFootstepPlan(std::string name)
{
  bool loadingNewPlan = (plan.name != name);
  double initHeight = (plan.name.length() > 0) ? plan.supportContact().p().z() : 0.;
  FootstepPlan defaultPlan = planInterpolator.getPlan(name);
  if(loadingNewPlan)
  {
    plan = defaultPlan;
    plan.name = name;
    mpc_.configure(mpcConfig_);
    if(!plan.mpcConfig.empty())
    {
      mpc_.configure(plan.mpcConfig);
    }
  }
  else // only reload contacts
  {
    plan.resetContacts(defaultPlan.contacts());
  }
  plan.complete(sole_);
  const sva::PTransformd & X_0_lf = controlRobot().surfacePose("LeftFootCenter");
  const sva::PTransformd & X_0_rf = controlRobot().surfacePose("RightFootCenter");
  plan.updateInitialTransform(X_0_lf, X_0_rf, initHeight);
  planInterpolator.worldReference(plan.initPose());
  planInterpolator.updateSupportPath(X_0_lf, X_0_rf);
  plan.rewind();
  if(loadingNewPlan)
  {
    LOG_INFO("Loaded footstep plan \"" << name << "\"");
  }
}

void Controller::startLogSegment(const std::string & label)
{
  if(segmentName_.length() > 0)
  {
    stopLogSegment();
  }
  segmentName_ = "t_" + std::to_string(++nbLogSegments_).erase(0, 1) + "_" + label;
  logger().addLogEntry(segmentName_, [this]() { return ctlTime_; });
}

void Controller::stopLogSegment()
{
  logger().removeLogEntry(segmentName_);
  segmentName_ = "";
}

bool Controller::updatePreview()
{
  mpc_.initState(pendulum());
  mpc_.comHeight(plan.comHeight());
  if(mpc_.buildAndSolve())
  {
    preview = mpc_.solution();
    return true;
  }
  else
  {
    nbMPCFailures_++;
    return false;
  }
}
} // namespace lipm_walking
