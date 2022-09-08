#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>

#include <mc_plugin_footstep_plan_msgs/FootStep.h>
#include <mc_plugin_footstep_plan_msgs/FootStepPlan.h>
#include <mc_plugin_footstep_plan_msgs/FootStepPlanRequest.h>
#include <mc_plugin_footstep_plan_msgs/FootStepPlanRequestService.h>
#include <ros/ros.h>

#include <ExternalFootstepPlanner/OnlineFootstepPlanner.h>
#include <chrono>
#include <thread>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

using FootStepPlanRequestService = mc_plugin_footstep_plan_msgs::FootStepPlanRequestService;
using FootStep = mc_plugin_footstep_plan_msgs::FootStep;

OnlineFootstepPlanner::OnlineFootstepPlanner() {}

OnlineFootstepPlanner::~OnlineFootstepPlanner()
{
  deactivate();
}

void OnlineFootstepPlanner::configure(const mc_rtc::Configuration & config)
{
  config("rate", rate_);
  config("verbose", verbose_);
  config("ignore_skipped_requests", ignore_skipped_requests_);
}

void OnlineFootstepPlanner::activate()
{
  if(activated_) return;
  if(verbose_) mc_rtc::log::info("[{}] activating...", name());
  run_ = true;
  rosThread_ = std::thread(&OnlineFootstepPlanner::rosThread, this);
  activated_ = true;
  if(verbose_) mc_rtc::log::success("[{}] activated", name());
}

void OnlineFootstepPlanner::deactivate()
{
  if(!activated_) return;
  if(verbose_) mc_rtc::log::info("[{}] deactivating...", name());
  run_ = false;
  if(rosThread_.joinable())
  {
    rosThread_.join();
  }
  activated_ = false;
  if(verbose_) mc_rtc::log::success("[{}] deactivated", name());
}

void OnlineFootstepPlanner::rosThread()
{
  mc_rtc::log::info("[{}] Planner thread started", name());
  auto & nh = *mc_rtc::ROSBridge::get_node_handle();
  // Service to request generation of a footstep plan
  // XXX: calling it should cancel the previous ongoing request (this is not the case in OnlineFootstepPlanner)
  ros::ServiceClient footstep_service = nh.serviceClient<FootStepPlanRequestService>(footstep_service_topic_);
  mc_rtc::log::info("[{}] Waiting for service \"{}\"", name(), footstep_service_topic_);

  auto futurePlan = std::future<boost::optional<Plan>>{};

  ros::Rate rate(rate_);
  bool was_available_ = false;
  while(ros::ok() && run_)
  {
    available_ = footstep_service.exists();
    if(!available_)
    {
      if(was_available_)
      {
        mc_rtc::log::warning("[{}] Service \"{}\" is no longer available", name(), footstep_service_topic_);
        was_available_ = false;
      }
      continue;
    }
    else if(!was_available_) // if service becomes available
    {
      mc_rtc::log::info("[{}] Service \"{}\" is now available", name(), footstep_service_topic_);
      was_available_ = true;
    }
    if(planRequested_)
    {
      planReceived_ = false;
      // Service should only trigger plan computation in a thread
      // This plan computation should be cancellable if a new plan request comes in
      // Do we really want to use the existing ROS services?
      // Direct integration with the planner sounds better here.

      // XXX for now calling the service will be blocking and non cancellable
      // - We need a service that just triggers plan computation and publishes the result on another topic (with request
      // id tracking to make sure we have the correct answer)
      //   but this doesn't make it cancellable
      // - Or we need to integrate the planner here directly
      futurePlan = std::async([this, &footstep_service]() -> boost::optional<Plan> {
        auto request = Request{};
        {
          std::lock_guard<std::mutex> lock{requestMutex_};
          request = request_;
        }
        auto req = FootStepPlanRequestService{};
        auto assign_pose = [](const SE2d & goal, geometry_msgs::Pose2D & msg) {
          msg.x = goal.x;
          msg.y = goal.y;
          msg.theta = goal.theta;
        };
        req.request.start_left.leg = FootStep::LEFT;
        assign_pose(request.start_left_foot, req.request.start_left.pose);
        req.request.start_right.leg = FootStep::RIGHT;
        assign_pose(request.start_right_foot, req.request.start_right.pose);
        req.request.end_left.leg = FootStep::LEFT;
        assign_pose(request.goal_left_foot, req.request.end_left.pose);
        req.request.end_right.leg = FootStep::RIGHT;
        assign_pose(request.goal_right_foot, req.request.end_right.pose);
        req.request.supportLeg = static_cast<uint8_t>(request.support_foot);
        req.request.allowed_time = request.allowed_time;
        auto res = footstep_service.call(req);
        planRequested_ = false;
        auto optplan = boost::optional<Plan>{};
        if(res)
        {
          Plan plan;
          for(const auto & footstep : req.response.plan)
          {
            plan.contacts.emplace_back(static_cast<Foot>(footstep.leg),
                                       SE2d{footstep.pose.x, footstep.pose.y, footstep.pose.theta});
          }
          optplan = plan;
        }
        else
        {
          mc_rtc::log::error("[{}] Service call failed, no new plan available", name());
        }
        return optplan;
      });
    }

    if(futurePlan.valid())
    {
      auto plan = futurePlan.get();
      if(plan)
      {
        if(verbose_)
        {
          mc_rtc::log::info("[{}] Received plan:\n{}", name(), plan.get());
        }
        {
          std::lock_guard<std::mutex> lock{planMutex_};
          plan_ = plan.get();
          planReceived_ = true;
        }
      }
    }
    rate.sleep();
  }
  mc_rtc::log::info("[{}] Planner thread stopped", name());
}

void OnlineFootstepPlanner::requestPlan(const Request & request)
{
  // request plan in ROS thread
  // Or set up planner here directly
  if(verbose_) mc_rtc::log::info("[{}] Requesting plan", name());
  if(planRequested_)
  {
    if(!ignore_skipped_requests_)
      mc_rtc::log::warning("[{}] Couldn't request plan as a plan is already being computed by the planner", name());
  }
  else
  {
    std::lock_guard<std::mutex> lock{requestMutex_};
    request_ = request;
    planRequested_ = true;
  }
}

/**
 * @brief Checks whether the last plan request suceeded
 *
 * @return true if the last plan request has successfully returned a plan
 * @return false otherwise
 */
bool OnlineFootstepPlanner::hasPlan() const noexcept
{
  return planReceived_;
}

Plan OnlineFootstepPlanner::popPlan()
{
  if(verbose_) mc_rtc::log::info("[{}] Getting plan", name());
  std::lock_guard<std::mutex> lock{planMutex_};
  planReceived_ = false;
  return std::move(plan_); // plan_ is invalid after this
}

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin
