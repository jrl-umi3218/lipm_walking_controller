#include <mc_control/GlobalPluginMacros.h>
#include <mc_filter/utils/clamp.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/ros.h>

#include <ExternalFootstepPlanner/DummyPlanner.h>
#include <ExternalFootstepPlanner/ExternalFootstepPlannerPlugin.h>
#ifdef USE_ONLINE_FOOTSTEP_PLANNER
#  include <ExternalFootstepPlanner/OnlineFootstepPlanner.h>
#endif
#ifdef USE_HYBRID_PLANNER
#  include <ExternalFootstepPlanner/HybridPlanner.h>
#endif

namespace mc_plugin
{

namespace ExternalFootstepPlanner
{

void ExternalFootstepPlannerPlugin::init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config)
{
#ifdef USE_ONLINE_FOOTSTEP_PLANNER
  supportedPlanners_.emplace_back("OnlineFootstepPlanner");
#endif
#ifdef USE_HYBRID_PLANNER
  supportedPlanners_.emplace_back("HybridPlanner");
#endif

  using namespace mc_rtc::gui;
  auto & ctl = gc.controller();
  this->gui_ = ctl.gui();
  config_ = config;
  config("category", category_);
  if(config.has("velocity_target"))
  {
    if(config("velocity_target").has("planning_distance"))
    {
      std::array<double, 3> distance = config("velocity_target")("planning_distance");
      setLocalVelocityPlanningDistance({distance[0], distance[1], distance[2]});
    }
  }

  config("planner", plannerName_);
  changePlanner(gc, plannerName_);

  /* Tsuru add */
  if(config.has("default_target_type"))
  {
    targetType_ = (std::string)config("default_target_type");
    mc_rtc::log::success("[{}] set target_type: {}", name(), targetType_);
  }

  ctl.datastore().make_call("ExternalFootstepPlanner::Available", [this]() { return planner_->available(); });
  ctl.datastore().make_call("ExternalFootstepPlanner::Activate", [this, &gc]() { activate(gc); });
  ctl.datastore().make_call("ExternalFootstepPlanner::Deactivate", [this]() { deactivate(); });
  // Do we need replanning?
  ctl.datastore().make_call("ExternalFootstepPlanner::PlanningRequested", [this]() {
    return worldPositionTargetChanged_ || localPositionTargetChanged_ || request_hybrid_plan_;
  }); // joystick input uses "localPositionTargetChanged_"
  ctl.datastore().make_call("ExternalFootstepPlanner::WorldPositionTargetChanged",
                            [this]() { return worldPositionTargetChanged_; });
  ctl.datastore().make_call("ExternalFootstepPlanner::WorldPositionTarget",
                            [this]() -> const SE2d & { return worldPositionTarget_; });
  ctl.datastore().make_call("ExternalFootstepPlanner::LocalPositionTargetChanged",
                            [this]() { return localPositionTargetChanged_; });
  ctl.datastore().make_call("ExternalFootstepPlanner::LocalPositionTarget",
                            [this]() -> const SE2d & { return localPositionTarget_; });

  ctl.datastore().make_call("ExternalFootstepPlanner::SetTargetType",
                            [this](const std::string & targetType) { changeTargetType(targetType); });
  ctl.datastore().make_call("ExternalFootstepPlanner::SetWorldPositionTarget",
                            [this](const SE2d & worldTarget) { setWorldPositionTarget(worldTarget); });
  ctl.datastore().make_call("ExternalFootstepPlanner::SetLocalPositionTarget",
                            [this](const SE2d & localTarget) { setLocalPositionTarget(localTarget); });
  ctl.datastore().make_call("ExternalFootstepPlanner::SetLocalVelocityTarget",
                            [this](const SE2d & localVelocity) { setLocalVelocityTarget(localVelocity); });
  ctl.datastore().make_call("ExternalFootstepPlanner::RequestHybridPlan", [this]() { request_hybrid_plan_ = true; });
  /* Tsuru add */
  ctl.datastore().make_call(
      "ExternalFootstepPlanner::SetJoystickVelocityTarget",
      [this](const sensor_msgs::Joy & joystickInput) { setJoystickVelocityTarget(joystickInput); });

  // Call this to request a new plan
  ctl.datastore().make_call("ExternalFootstepPlanner::RequestPlan", [this](const Request & request) {
    worldPositionTargetChanged_ = false;
    localPositionTargetChanged_ = false;
    request_hybrid_plan_ = false;
    planner_->requestPlan(request);
  });
  ctl.datastore().make_call("ExternalFootstepPlanner::HasPlan", [this]() { return planner_->hasPlan(); });
  ctl.datastore().make_call("ExternalFootstepPlanner::PopPlan", [this]() { return planner_->popPlan(); });

  mc_rtc::log::success("[{}] Plugin initialized", name());
  reset(gc);
}

void ExternalFootstepPlannerPlugin::reset(mc_control::MCGlobalController &)
{
  mc_rtc::log::success("[{}] Plugin reset", name());
}

void ExternalFootstepPlannerPlugin::before(mc_control::MCGlobalController & /* gc */)
{
  if(wasAvailable_ && !planner_->available())
  {
    removePlannerGUI();
    wasAvailable_ = false;
  }
  else if(activated_ && !wasAvailable_ && planner_->available())
  {
    addPlannerGUI();
    wasAvailable_ = true;
  }
}

mc_control::GlobalPlugin::GlobalPluginConfiguration ExternalFootstepPlannerPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = false;
  return out;
}

bool ExternalFootstepPlannerPlugin::plannerSupported(const std::string & plannerName) const noexcept
{
  return std::find(supportedPlanners_.begin(), supportedPlanners_.end(), plannerName) != supportedPlanners_.end();
}

void ExternalFootstepPlannerPlugin::changePlanner(mc_control::MCGlobalController & gc, const std::string & plannerName)
{
  mc_rtc::log::info("[{}] Requested planner \"{}\"", name(), plannerName);
  // Do nothing if this planner is already being used
  if(plannerName == plannerName_ && planner_)
  {
    return;
  }

  if(!plannerSupported(plannerName_))
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[{}] does not support planner {} (supported planners are: {})",
                                                        name(), plannerName, mc_rtc::io::to_string(supportedPlanners_));
  }

  deactivate();

  if(plannerName == "OnlineFootstepPlanner")
  {
#ifdef USE_ONLINE_FOOTSTEP_PLANNER
    planner_.reset(new OnlineFootstepPlanner{});
    if(config_.has("OnlineFootstepPlanner"))
    {
      planner_->configure(config_("OnlineFootstepPlanner"));
    }
#endif
  }
  else if(plannerName == "HybridPlanner")
  {
    //#ifdef USE_HYBRID_PLANNER
    planner_.reset(new HybridPlanner{gc.controller()});
    planner_->configure(config_);
    //#endif
  }

  if(plannerName == "DummyPlanner")
  {
    planner_.reset(new DummyPlanner{});
  }

  mc_rtc::log::success("[{}] Changed planner from {} to {}", name(), plannerName_, plannerName);
  plannerName_ = plannerName;
}

void ExternalFootstepPlannerPlugin::setWorldPositionTarget(const SE2d & worldTarget)
{
  worldPositionTarget_ = worldTarget;
  worldPositionTargetChanged_ = true;
}

void ExternalFootstepPlannerPlugin::setLocalPositionTarget(const SE2d & localTarget)
{
  localPositionTarget_ = localTarget;
  localPositionTargetChanged_ = true;
}

void ExternalFootstepPlannerPlugin::setLocalVelocityTarget(const SE2d & localVelocity)
{
  setLocalPositionTarget(localVelocity);
}

void ExternalFootstepPlannerPlugin::setLocalVelocityPlanningDistance(const SE2d & distance)
{
  planningDistance_.x = std::fabs(distance.x);
  planningDistance_.y = std::fabs(distance.y);
  planningDistance_.theta = std::fabs(distance.theta);
  mc_filter::utils::clampInPlace(localVelocityTarget_.x, -planningDistance_.x, planningDistance_.x);
  mc_filter::utils::clampInPlace(localVelocityTarget_.y, -planningDistance_.y, planningDistance_.y);
  mc_filter::utils::clampInPlace(localVelocityTarget_.theta, -planningDistance_.theta, planningDistance_.theta);
  setLocalVelocityTarget(localVelocityTarget_);
}

/* Tsuru add */
void ExternalFootstepPlannerPlugin::setJoystickVelocityTarget(const sensor_msgs::Joy & joystickInput)
{
  SE2d localVelocity;

  /* convert sensor_msgs::Joy -> SE2d */
  if(targetType_ == "PS4 Controller")
    localVelocity = InputConvertor::convert_PS4_to_SE2d(joystickInput);

  else if(targetType_ == "Oculus Controller")
    localVelocity = InputConvertor::convert_Oculus_to_SE2d(joystickInput);

  setLocalVelocityTarget(localVelocity);
  return;
}

void ExternalFootstepPlannerPlugin::changeTargetType(const std::string & targetType)
{
  if(!planner_) return;

  using namespace mc_rtc::gui;
  auto & gui = *gui_;
  std::vector<std::string> category = category_;
  category.push_back("Target");
  gui.removeCategory(category);

  /* if Joystick Thread has already started once before, stop it here. */
  if(run_)
  {
    run_ = false;
    if(joystickSubscribeThread_.joinable())
    {
      joystickSubscribeThread_.join();
    }
  }

  if(targetType == "World SE2")
  {
    gui.addElement(this, category,
                   XYTheta(
                       "World target [m, rad]",
                       [this]() -> std::array<double, 4> {
                         return {worldPositionTarget_.x, worldPositionTarget_.y, worldPositionTarget_.theta, 0.};
                       },
                       [this](const std::array<double, 4> & target) {
                         setWorldPositionTarget({target[0], target[1], target[2]});
                       }));
  }
  else if(targetType == "Local SE2")
  {
    gui.addElement(this, category,
                   ArrayInput(
                       "Local target [m, rad]",
                       [this]() -> std::array<double, 3> {
                         return {localPositionTarget_.x, localPositionTarget_.y, localPositionTarget_.theta};
                       },
                       [this](const std::array<double, 3> & target) {
                         setLocalPositionTarget({target[0], target[1], target[2]});
                       }));
  }
  else if(targetType == "Local Velocity")
  {
    mc_rtc::log::warning("[{}] Local Veloctity target is not implemented yet.", name());
    auto makeSliders = [this, category, &gui]() {
      gui.removeElement(category, "Local Velocity [x]");
      gui.removeElement(category, "Local Velocity [y]");
      gui.removeElement(category, "Local Velocity [theta]");
      gui.addElement(this, category,
                     NumberSlider(
                         "Local Velocity [x]", [this]() { return localVelocityTarget_.x; },
                         [this](double vx) {
                           localVelocityTarget_.x = vx;
                           setLocalVelocityTarget(localVelocityTarget_);
                         },
                         -planningDistance_.x, planningDistance_.x),
                     NumberSlider(
                         "Local Velocity [y]", [this]() { return localVelocityTarget_.y; },
                         [this](double vy) {
                           localVelocityTarget_.y = vy;
                           setLocalVelocityTarget(localVelocityTarget_);
                         },
                         -planningDistance_.y, planningDistance_.y),
                     NumberSlider(
                         "Local Velocity [theta]", [this]() { return localVelocityTarget_.theta; },
                         [this](double vy) {
                           localVelocityTarget_.theta = vy;
                           setLocalVelocityTarget(localVelocityTarget_);
                         },
                         -planningDistance_.theta, planningDistance_.theta));
    };

    makeSliders();
    gui.addElement(this, category,
                   ArrayInput(
                       "Planning Distance", {"x [m]", "y [m]", "theta [rad]"},
                       [this]() -> std::array<double, 3> {
                         return {planningDistance_.x, planningDistance_.y, planningDistance_.theta};
                       },
                       [this, makeSliders](const std::array<double, 3> & d) {
                         setLocalVelocityPlanningDistance({d[0], d[1], d[2]});
                         makeSliders();
                       }));
  }
  else if(targetType == "PS4 Controller" || targetType == "Oculus Controller")
  {
    gui.addElement(this, category, Label("is Controller Connected?", [this]() { return isControllerConnected_; }));
    // activate a new ROS thread
    run_ = true;
    joystickSubscribeThread_ = std::thread(&ExternalFootstepPlannerPlugin::joystickSubscribeThread, this);
  }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[{}] Target type {} is not supported (supported: {})", name(),
                                                        mc_rtc::io::to_string(supportedTargetTypes_));
    return;
  }
  targetType_ = targetType;
}

void ExternalFootstepPlannerPlugin::activate(mc_control::MCGlobalController & gc)
{
  if(activated_) return;

  using namespace mc_rtc::gui;
  auto & gui = *gui_;
  gui.addElement(this, category_,
                 ComboInput(
                     "Planner", supportedPlanners_, [this]() { return plannerName_; },
                     [this, &gc](const std::string & planner) {
                       changePlanner(gc, planner);
                       activate(gc);
                     }));
  gui.addElement(this, category_, Label("Available?", [this]() { return planner_->available(); }));
  planner_->activate();

  activated_ = true;
}

void ExternalFootstepPlannerPlugin::deactivate()
{
  if(!activated_) return;
  using namespace mc_rtc::gui;
  auto & gui = *gui_;
  planner_->deactivate();
  gui.removeElements(this);

  /* Tsuru add below to Subscribe Joystick Input through ROS topic. */
  run_ = false;
  if(joystickSubscribeThread_.joinable())
  {
    joystickSubscribeThread_.join();
  }

  activated_ = false;
  wasAvailable_ = false;
}

void ExternalFootstepPlannerPlugin::addPlannerGUI()
{
  using namespace mc_rtc::gui;
  auto & gui = *gui_;
  gui.addElement(this, category_,
                 ComboInput(
                     "Target type", supportedTargetTypes_, [this]() { return targetType_; },
                     [this](const std::string & targetType) { changeTargetType(targetType); }));
  changeTargetType(targetType_);
}

void ExternalFootstepPlannerPlugin::removePlannerGUI()
{
  using namespace mc_rtc::gui;
  auto & gui = *gui_;
  auto category = category_;
  category.push_back("Target");
  gui.removeCategory(category);
  gui.removeElement(category_, "Target type");
}

void ExternalFootstepPlannerPlugin::joystickSubscribeThread()
{
  mc_rtc::log::info("[{}] Joystick subscribe thread started", name());
  auto & nh = *mc_rtc::ROSBridge::get_node_handle();
  // Service to request generation of a footstep plan
  // XXX: calling it should cancel the previous ongoing request (this is not the case in OnlineFootstepPlanner)

  ros::Subscriber ps4_sub =
      nh.subscribe<sensor_msgs::Joy>(joystick_topic_, 1, &ExternalFootstepPlannerPlugin::joystick_callback, this);

  ros::Rate rate(rate_);
  while(ros::ok() && run_)
  {
    /* * * * * * * * * * * * */
    /* Receive Joystic Input */
    /* * * * * * * * * * * * */
    // if(controller is available)
    ros::spinOnce(); // for Joystick callback function
    rate.sleep();
  }
  mc_rtc::log::info("[{}] Joystick subscribe thread stopped", name());
}

void ExternalFootstepPlannerPlugin::joystick_callback(const sensor_msgs::JoyConstPtr & joystick_input)
{
  // ROS_WARN("joystick callback start");
  // ROS_WARN("%1.2f, %1.2f, %1.2f, %1.2f", joystick_input->axes.at(0), joystick_input->axes.at(1),
  //          joystick_input->axes.at(2), joystick_input->axes.at(3));
  /* update the LocalTarget with the latest Joy message */
  setJoystickVelocityTarget(*joystick_input);
  return;
}

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ExternalFootstepPlannerPlugin", mc_plugin::ExternalFootstepPlanner::ExternalFootstepPlannerPlugin)
