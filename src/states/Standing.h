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

#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <lipm_walking/Controller.h>
#include <lipm_walking/State.h>

namespace lipm_walking
{

namespace states
{

/** Enable stabilizer and keep CoM at a reference position.
 *
 * Applies a simple CoM set-point task:
 *
 * \f[
 *   \ddot{c} = K (c^d - c) - B \dot{c}
 * \f]
 *
 * with critical damping \f$B = 2 \sqrt{K}\f$.
 *
 */
struct Standing : State
{
  void configure(const mc_rtc::Configuration &) override;

  /** Start state.
   *
   */
  void start() override;

  /** Teardown state.
   *
   */
  void teardown() override;

  /** Check for footstep plan updates.
   *
   */
  void checkPlanUpdates();

  /** Check transitions at beginning of control cycle.
   *
   */
  bool checkTransitions() override;

  /** Main state function, called if no transition at this cycle.
   *
   */
  void runState() override;

  /** Update target CoM and CoP.
   *
   * \param leftFootRatio Left foot weight index between 0 and 1.
   *
   */
  void updateTarget(double leftFootRatio);

  /** Enable startWalking_ boolean and update GUI.
   *
   */
  void startWalking();

protected:
  /** Handle requesting/receiving external plans */
  void handleExternalPlan();

private:
  Contact leftFootContact_; /**< Current left foot contact handle in plan */
  Contact rightFootContact_; /**< Current right foot contact handle in plan */
  Eigen::Vector3d copTarget_; /**< CoP target computed from GUI input */
  bool autoplay_ = false; /**< Has the user clicked on "Start walking"? */
  std::vector<std::string> autoplay_plans_; /** Plans to play if config(autoplay) = true */
  double leftFootRatio_; /**< Left foot ratio from GUI input */

  /* Tsuru add */
  lipm_walking::Contact supportContact_; // Keep the last contact foot stably.
  lipm_walking::Contact targetContact_; // Keep the last contact foot stably.
};

} // namespace states

} // namespace lipm_walking
