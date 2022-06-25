/**
 * @file LibCheck.h
 *
 * This file defines a representation that checks some behavior control properties
 *
 * @author Daniel Krause
 */

#pragma once

#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Function.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

STREAMABLE(LibCheck,
{
  ENUM(CheckedOutput,
  {,
    motionRequest,
    headMotionRequest,
    activity,
    passTarget,
    firstTeamCheckedOutput,
    teamActivity = firstTeamCheckedOutput,
    timeToReachBall,
    teammateRoles,
    role,
  });

  /** Increments one counter */
  FUNCTION(void(LibCheck::CheckedOutput outputToCheck)) inc;

  /** Indicates that an arm has been set */
  FUNCTION(void(Arms::Arm arm)) setArm;

  /** Checks whether an arm has been set */
  FUNCTION(bool(Arms::Arm arm)) wasSetArm;

  /** Performs checks for the individual behavior */
  FUNCTION(void(const MotionRequest& theMotionRequest)) performCheck;

  /** Performs checks for the team behavior */
  FUNCTION(void()) performTeamCheck,
  
  (int)(0) closerToTheBall,
  (bool)(false) LeftAttacking,
  (bool)(false) LeftDefending,
  (bool)(false) RightAttacking,
  (bool)(false) RightDefending,
  (bool)(false) positionToPass,
  (int)(0) TeammateFallenNumber,
  (bool)(false) TeammateObstacleAvoid,
  (bool)(false) OpponentObstacle,
  (bool)(false) TeammateSeeingBall,
  (bool)(false) TimeToSendMessage,
});
