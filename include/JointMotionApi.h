#pragma once

#include "JointPlanner.h"

// Firmware-level entry points shared by the local console and JointBus.
// Transport parsing and response formatting remain outside JointPlanner.
JointMoveOutcome moveJointToDeg(float targetZeroedDeg);
JointMoveOutcome jointMoveTo(float targetDeg, float vmaxDegS, float amaxDegS2);
JointMoveOutcome jointMoveToBlended(float targetDeg, float vmaxDegS, float amaxDegS2);
JointMoveOutcome jointStop();
