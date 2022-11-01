/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>

#include <sensor_msgs/Joy.h>

#include <ExternalFootstepPlanner/ExternalFootstepPlanner.h>
#include <ExternalFootstepPlanner/ExternalFootstepPlannerPlugin.h>
#include <ExternalFootstepPlanner/InputConvertor.h>
#include <ExternalFootstepPlanner/SE2d.h>

namespace mc_plugin
{

namespace ExternalFootstepPlanner
{

/**
 * @brief This plugin handles external footstep planning

 * It currently supports position-based planners, that expect as input:
 * - The initial support feet state (x, y, theta)
 * - The final support feet state (x, y, theta)
 *
 * Communication between the plugin and your controller is done through the datastore.
 *
 * Currently supports:
 * - OnlineFootstepPlanner: https://github.com/isri-aist/OnlineFootstepPlanner
 *   This planner is a modified version of the humanoid_navigation ROS stack.
 *   It supports planning with obstacles avoidance, online replanning and more, and can be
 *   controlled through ROS services.
 *   See also:
 *   - The OnlineFootstepPlanner project
 *   - @see OnlineFootstepPlanner
 */
struct ExternalFootstepPlannerPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;
  void reset(mc_control::MCGlobalController & controller) override;
  void before(mc_control::MCGlobalController &) override;
  void after(mc_control::MCGlobalController &) override {}
  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  /**
   * @brief Name of this plugin
   *
   * @return const std::string&
   */
  inline const std::string & name() const noexcept
  {
    return name_;
  }

protected:
  /**
   * @brief Change which footstep planner is used
   *
   * @param plannerName Name of the desired planner. Must be one of supportPlanners_.
   */
  void changePlanner(mc_control::MCGlobalController & gc, const std::string & plannerName);

  /**
   * @brief Change type of target used (world, local, velocity, etc)
   *
   * @param targetType Desired target type. Must be one of supportedTargetTypes_.
   */
  void changeTargetType(const std::string & targetType);

  /**
   * @brief Set the desired world target
   *
   * @param worldTarget Target in world frame (X;Y;Theta)
   */
  void setWorldPositionTarget(const SE2d & worldTarget);

  /**
   * @brief Set the desired local target
   * @note the local frame is defined as the frame halfway in-between the support feet at the start of the plan.
   * @param worldTarget Target in local frame (X;Y;Theta)
   */

  void setLocalPositionTarget(const SE2d & localTarget);

  /**
   * @brief Set the Local Velocity Target object
   * @note For now this is implemented as requesting a local target as none of the supported planners support velocity
   * inputs. Also the planner does not have suitable input parameters to match the footstep length to the desired
   * velocity. As a result "velocity" here is interpreted in an arbitrary unity and should be seen as a desired
   * direction.
   *
   * @note Historical background: this is a direct replacement for the deprecated input_convertor_node
   *
   * @param velocityTarget Desired velocity in local frame
   * @see setLocalPositionTarget
   */
  void setLocalVelocityTarget(const SE2d & velocityTarget);

  /**
   * @brief Sets the maximal distance away from the local frame that we ask the planner for
   *
   * This parameter has a significant influence in how the planner behaves. In particular setting a short distance leads
   * to more natural "velocity control" behaviour, while setting a large distance makes the planning less responsive.
   *
   * @param distance
   */
  void setLocalVelocityPlanningDistance(const SE2d & distance);

  /**
   * @brief [Tsuru add] Sets the Local Velocity Target object by ROS Joystick input
   * @note For now this is implemented as requesting a local target as none of the supported planners support velocity
   * inputs. Also the planner does not have suitable input parameters to match the footstep length to the desired
   * velocity. As a result "velocity" here is interpreted in an arbitrary unity and should be seen as a desired
   * direction.
   *
   * @note Historical background: this is a direct replacement for the deprecated input_convertor_node
   *
   * @param joystickInput ROS message for biped walking in local frame
   * @see setLocalPositionTarget
   */
  void setJoystickVelocityTarget(const sensor_msgs::Joy & joystickInput);

  /**
   * @brief Add GUI elements that are visible when the planner is available
   */
  void addPlannerGUI();
  /**
   * @brief Remove elements added by addPlannerGUI()
   */
  void removePlannerGUI();
  /**
   * @brief Activates the plugin (add it to the GUI) and the planner
   */
  void activate(mc_control::MCGlobalController & gc);
  /**
   * @brief Deactivate the plugin and the planner
   *
   * Can be reactivated by activate()
   */
  void deactivate();

  /**
   * @brief Check if the desired planner is supported
   *
   * @param plannerName Name of the planner
   *
   * @return True if the planner is supported
   */
  bool plannerSupported(const std::string & plannerName) const noexcept;

protected:
  std::shared_ptr<mc_rtc::gui::StateBuilder> gui_{nullptr};
  bool activated_ = false; ///< Whether this plugin is active
  mc_rtc::Configuration config_; ///< Stores the plugin configuration, in particular the planners' options
  std::string name_{"ExternalFootstepPlanner"}; ///< Name of the plugin (mainly used for logging)
  std::vector<std::string> category_{"ExternalFootStepPlanner"}; ///< Category in the gui

  std::vector<std::string> supportedPlanners_{"DummyPlanner"}; ///< List of supported planners
  std::string plannerName_{"DummyPlanner"}; ///< Name of the currently activated planner
  std::unique_ptr<ExternalFootstepPlanner> planner_{nullptr}; ///< Planner implementation
  bool wasAvailable_ = false; ///< True if the planner was active during the previous iteration

  std::vector<std::string> supportedTargetTypes_{"World SE2", "Local SE2", "Local Velocity", "PS4 Controller",
                                                 "Oculus Controller"};
  std::string targetType_{"World SE2"};

  bool worldPositionTargetChanged_ =
      false; ///< True if the requested target has changed (either through the GUI or other means)
  SE2d worldPositionTarget_{}; ///< Requested target

  bool localPositionTargetChanged_{false};
  SE2d localPositionTarget_{};

  SE2d localVelocityTarget_{};
  SE2d planningDistance_{0.3, 0.3, mc_rtc::constants::PI / 2}; // How far ahead should we plan?

  /* Tsuru add */
  std::thread joystickSubscribeThread_;

  bool request_hybrid_plan_ = false;

protected:
  void joystickSubscribeThread();
  void joystick_callback(const sensor_msgs::JoyConstPtr & joystick_input);
  std::string joystick_topic_ = "/avatar/joy";
  double rate_ = 30;
  bool run_{false};
  bool joystickPositionTargetChanged_{false};
  bool isControllerConnected_{false};
};

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin
