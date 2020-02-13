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
  /** Start state. */
  void start() override;

  /** Teardown state. */
  void teardown() override;

  /** Main state function, called if no transition at this cycle. */
  void runState() override;

  bool checkTransitions() override;

  /** Enable startWalking_ boolean and update GUI.
   *
   */
  void startWalking();

protected:
  void checkPlanUpdates();

  /** Change footstep plan.
   *
   * \param name New plan name.
   *
   */
  void updatePlan(const std::string & name);

private:
  bool planChanged_; /**< Has footstep plan changed? */
  bool startWalking_; /**< Has the user clicked on "Start walking"? */
  unsigned lastInterpolatorIter_; /**< Last iteration number of the plan interpolator */
};

} // namespace states

} // namespace lipm_walking
