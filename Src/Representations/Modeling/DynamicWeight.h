/**
 * @file LibCodeRelease.h
 */

#pragma once
#include "Tools/Math/Pose2f.h"
#include "Tools/Function.h"

STREAMABLE(DynamicWeight,
{,
  (float) ballWeight,
  (float) areaWeight,
  (float) cornerWeight,
  (float) arcoWeight,
});