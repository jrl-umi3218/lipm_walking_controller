#include <mc_rtc/logging.h>

#include <ExternalFootstepPlanner/ExternalFootstepPlanner.h>
#include <ExternalFootstepPlanner/HybridPlanner.h>
#include <lipm_walking/utils/SE2d.h>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

HybridPlanner::HybridPlanner(mc_control::MCController & ctl) : ctl_(ctl)
{
  mc_rtc::log::success("[HybridPlanner] created");
}

HybridPlanner::~HybridPlanner()
{
  // deactivate();
}

void HybridPlanner::configure(const mc_rtc::Configuration & config)
{
  auto conf = config("HybridPlanner", mc_rtc::Configuration{});
  conf.load(ctl_.config()("HybridPlanner"));
  mc_rtc::log::info("[HybridPlanner] Requesting configuration: {}", conf.dump(true));
  ctl_.datastore().assign<mc_rtc::Configuration>("footsteps_planner::planner_config", conf);
  Tp_ = conf("Tp");
  delta_ = conf("delta");
}

void HybridPlanner::activate()
{
  ctl_.datastore().make_call("HybridPlanner::SetVelocity",
                             [this](const Eigen::Vector3d & velocity) { setVelocity(velocity); });
  ctl_.datastore().make_call("HybridPlanner::GetVelocity", [this]() { return velocity_; });
  ctl_.gui()->addElement({"Hybrid Footstep Planner"},
                         mc_rtc::gui::Label("Support Foot", [this]() { return supportFootName_; } ),
                         mc_rtc::gui::ArrayInput(
                             "Velocity", [this]() -> const Eigen::Vector3d & { return velocity_; },
                             [this](const Eigen::Vector3d & vel) { this->setVelocity(vel); }));
}

void HybridPlanner::setVelocity(const Eigen::Vector3d & velocity)
{
  velocity_ = velocity;
  ctl_.datastore().call("ExternalFootstepPlanner::RequestHybridPlan");
}

void HybridPlanner::deactivate()
{
  // if(!activated_) return;
  // if(verbose_) mc_rtc::log::info("[{}] deactivating...", name());
  // run_ = false;
  // if(rosThread_.joinable())
  // {
  //   rosThread_.join();
  // }
  // activated_ = false;
  // if(verbose_) mc_rtc::log::success("[{}] deactivated", name());
}

void HybridPlanner::requestPlan(const Request & request)
{
  if(computing_) return; // do nothing if it's already computing
  computing_ = true;

  auto targetVel = velocity_;

  plan_.contacts.clear();

  sva::PTransformd X_0_support = sva::PTransformd::Identity();
  auto X_left = X_0_support =
      lipm_walking::utils::SE2d{request.start_left_foot.x, request.start_left_foot.y, request.start_left_foot.theta}
          .asPTransform();
  auto X_right =
      lipm_walking::utils::SE2d{request.start_right_foot.x, request.start_right_foot.y, request.start_right_foot.theta}
          .asPTransform();

  if(request.support_foot == Foot::Left)
  {
    supportFootName_ = "LeftFoot";
    X_0_support = X_left;
    // plan_.contacts.emplace_back(Foot::Left, request.start_left_foot);
  }
  else
  {
    supportFootName_ = "RightFoot";
    X_0_support = X_right;
    // plan_.contacts.emplace_back(Foot::Left, request.start_right_foot);
  }
  ctl_.datastore().assign<std::string>("footsteps_planner::support_foot_name", supportFootName_);

  // mc_rtc::log::info("Hybrid planner plan requested");

  // input velocity expressed in support foot frame
  auto & input_vel = ctl_.datastore().get<std::vector<sva::MotionVecd>>("footsteps_planner::input_vel");
  input_vel.clear();
  unsigned int p = static_cast<unsigned int>(Tp_ / delta_);
  // mc_rtc::log::info("P: {}", p);

  // start = support

  // if velocity_;
  if(request.support_foot == Foot::Left && velocity_.y() > 0)
  {
    targetVel.y() = 0;
  }
  else if(request.support_foot == Foot::Right && velocity_.y() < 0)
  {
    targetVel.y() = 0;
  }

  // Going at that speed for Tp_ (6 seconds)
  for(unsigned i = 0; i < p; ++i)
  {
    input_vel.push_back(
        sva::MotionVecd(Eigen::Vector3d{0, 0, targetVel.z()}, Eigen::Vector3d{targetVel.x(), targetVel.y(), 0}));
  }
  ctl_.datastore().assign<std::vector<sva::PTransformd>>("footsteps_planner::input_steps", {});
  // XXX use
  // request.goal_left_foot.x, y, z
  // request.goal_right_foot.x y z

  ctl_.datastore().assign<std::vector<sva::PTransformd>>("footsteps_planner::input_steps", {});
  ctl_.datastore().assign<sva::PTransformd>("footsteps_planner::support_foot_pose", X_0_support);

  ctl_.datastore().assign<std::vector<double>>("footsteps_planner::input_time_steps", {1, 2, 3});
  // Blocking call
  ctl_.datastore().call("footstep_planner::compute_plan");
}

/**
 * @brief Checks whether the last plan request suceeded
 *
 * @return true if the last plan request has successfully returned a plan
 * @return false otherwise
 */
bool HybridPlanner::hasPlan() const noexcept
{
  return ctl_.datastore().get<std::vector<sva::PTransformd>>("footsteps_planner::output_steps").size();
}

Plan HybridPlanner::popPlan()
{
  auto plan_steps = ctl_.datastore().get<std::vector<sva::PTransformd>>("footsteps_planner::output_steps");
  auto foot = Foot::Left;
  if(supportFootName_ == "LeftFoot")
  {
    foot = Foot::Right;
  }
  else
  {
    foot = Foot::Left;
  }
  // FIXME hardcoded
  int i = 0;
  int Nsteps = 5;
  for(const auto & step_pose : plan_steps)
  {
    if(i++ > Nsteps) break;
    plan_.contacts.emplace_back(foot, lipm_walking::utils::SE2d{step_pose}.vector());
    if(foot == Foot::Left)
    {
      foot = Foot::Right;
    }
    else
    {
      foot = Foot::Left;
    }
  }
  ctl_.datastore().get<std::vector<sva::PTransformd>>("footsteps_planner::output_steps").clear();
  computing_ = false;
  // mc_rtc::log::info("");
  // mc_rtc::log::info("Received plan: {}", plan_);
  // mc_rtc::log::info("");
  return plan_;
}

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin
