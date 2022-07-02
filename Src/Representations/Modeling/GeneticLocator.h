
/**
 * @file LibCodeRelease.h
 */

#pragma once
#include "Tools/Math/Pose2f.h"
#include "Tools/Function.h"

STREAMABLE(GeneticLocator,
{
  FUNCTION(Pose2f(const bool activates, const int ctime, const float ballWeight)) activation,

  (float) optimalX,
  (float) optimalY,
  (float) lastime,
  (int) Hmtimes,
});