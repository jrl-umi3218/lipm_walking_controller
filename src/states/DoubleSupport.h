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

/** Double support phase while walking.
 *
 */
struct DoubleSupport : State
{
  /** Start state.
   *
   */
  void start() override;

  /** Teardown state.
   *
   */
  void teardown() override;

  /** Check transitions at beginning of control cycle.
   *
   */
  bool checkTransitions() override;

  /**
   * Check whether the support estiblishement was good (force above threshold
   * for N iterations)
   * If not, stop during this DSP
   *
   * @return False as soon as the contact is considered invalid
   */
  bool checkInitialSupport();

  /** Main state function, called if no transition at this cycle.
   *
   */
  void runState() override;

  /** Update horizontal MPC preview.
   *
   */
  void updatePreview();

protected:
  void handleExternalPlan();

private:
  bool stopDuringThisDSP_; /**< Stop walking during this DSP */
  double duration_; /**< Total duration of the DSP in [s] */
  double initLeftFootRatio_; /**< Left foot ratio at the beginning of DSP */
  double remTime_; /**< Time remaining until the end of the phase */
  double stateTime_; /**< Time since the beginning of the DSP */
  double targetLeftFootRatio_; /**< Left foot ratio at the end of DSP */
  double timeSinceLastPreviewUpdate_; /**< Time count used to schedule MPC updates, in [s] */

  double minSupportForce_ = 50;
  bool goodInitialSupport_ = true;
  double maxAbortPercent_ = 0.1; // Percentage util which we still allow to stop

};

} // namespace states

} // namespace lipm_walking
