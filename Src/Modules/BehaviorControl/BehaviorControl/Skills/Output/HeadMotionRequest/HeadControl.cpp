/**
 * @file HeadControl.cpp
 *
 * This file implements head control skills.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include <cmath>

SKILL_IMPLEMENTATION(HeadControl,
{,
  IMPLEMENTS(LookAtAngles),
  IMPLEMENTS(LookAtPoint),
  IMPLEMENTS(LookForward),
  // IMPLEMENTS(LookAround),
  REQUIRES(LibCheck),
  MODIFIES(HeadMotionRequest),
});

class HeadControl : public HeadControlBase
{
  void execute(const LookAtAngles& p) override
  {
    setPanTiltRequest(p.camera, p.pan, p.tilt, p.speed);
  }

  void execute(const LookAtPoint& p) override
  {
    setTargetOnGroundRequest(p.camera, p.target, p.speed);
  }

  void execute(const LookForward& p) override
  {
    setPanTiltRequest(HeadMotionRequest::autoCamera, 0.f, 0.38f, 150_deg);
  }

  // void execute(const LookAround& p) override
  // {
  //   setPanTiltRequest(p.camera, p.panStart, p.tilt, p.speed);
  //   for(int i=0;i<1000;i++){
  //     int r = 2;
  //   }
  //   setPanTiltRequest(p.camera, p.panEnd, p.tilt, p.speed);
  // }

  void setPanTiltRequest(HeadMotionRequest::CameraControlMode camera, Angle pan, Angle tilt, Angle speed, bool stopAndGoMode = false)
  {
    theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
    theHeadMotionRequest.cameraControlMode = camera;
    theHeadMotionRequest.pan = pan;
    theHeadMotionRequest.tilt = tilt;
    theHeadMotionRequest.speed = speed;
    theHeadMotionRequest.stopAndGoMode = stopAndGoMode;
    theLibCheck.inc(LibCheck::headMotionRequest);
  }

  // void setLookAroundPanRequest(HeadMotionRequest::CameraControlMode camera, Angle panStart, Angle panEnd, Angle tilt, Angle speed, bool stopAndGoMode = false)
  // {
  //   theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
  //   theHeadMotionRequest.cameraControlMode = camera;
  //   theHeadMotionRequest.panStart = panStart;
  //   theHeadMotionRequest.panEnd = panEnd;
  //   theHeadMotionRequest.tilt = tilt;
  //   theHeadMotionRequest.speed = speed;
  //   theHeadMotionRequest.stopAndGoMode = stopAndGoMode;
  //   theLibCheck.inc(LibCheck::headMotionRequest);
  // }

  void setTargetOnGroundRequest(HeadMotionRequest::CameraControlMode camera, const Vector3f& target, Angle speed)
  {
    theHeadMotionRequest.mode = HeadMotionRequest::targetOnGroundMode;
    theHeadMotionRequest.cameraControlMode = camera;
    theHeadMotionRequest.target = target;
    theHeadMotionRequest.speed = speed;
    theHeadMotionRequest.stopAndGoMode = false;
    theLibCheck.inc(LibCheck::headMotionRequest);
  }
};

MAKE_SKILL_IMPLEMENTATION(HeadControl);
