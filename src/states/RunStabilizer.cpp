/*
 * Copyright (c) 2020, CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "RunStabilizer.h"

namespace lipm_walking
{

void states::RunStabilizer::start()
{
  auto & ctl = controller();
  ctl.solver().addTask(stabilizer());
  output("OK");
}

void states::RunStabilizer::runState() {}

void states::RunStabilizer::teardown()
{
  auto & ctl = controller();
  ctl.solver().removeTask(stabilizer());
}

bool states::RunStabilizer::checkTransitions()
{
  return true;
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("LIPMWalking::RunStabilizer", lipm_walking::states::RunStabilizer)
