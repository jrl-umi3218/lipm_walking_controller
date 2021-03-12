/*
 * Copyright (c) 2020, CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <lipm_walking/Controller.h>
#include <lipm_walking/State.h>

namespace lipm_walking
{

/** States of the controller's finite state machine.
 *
 */
namespace states
{

/**
 * Adds/removes the global stabilizer task to the QP
 */
struct RunStabilizer : State
{
  void start() override;
  void teardown() override;
  void runState() override;
  /// Always true
  bool checkTransitions() override;
};

} // namespace states

} // namespace lipm_walking
