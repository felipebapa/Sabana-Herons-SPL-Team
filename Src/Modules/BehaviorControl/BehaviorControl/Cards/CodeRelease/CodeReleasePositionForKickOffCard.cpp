/**
 * @file CodeReleasePositionForKickOffCard.cpp
 *
 * This file implements nothing.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

#include "Representations/Communication/RobotInfo.h"
#include "Representations/Modeling/RobotPose.h"

CARD(CodeReleasePositionForKickOffCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
  CALLS(PathToTarget),
  CALLS(WalkAtRelativeSpeed),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (Pose2f)(Pose2f(0,-4150,0)) KeeperPos,
    (Pose2f)(Pose2f(0,-3000,0)) Defender1Pos,
    (Pose2f)(Pose2f(0,-2500,1500)) Defender2Pos,
    (Pose2f)(Pose2f(0,-2500,-1500)) Defender3Pos,
    (Pose2f)(Pose2f(0,-1000,0)) StrikerPos,
    (int)(100) StopThreshold,
    (float)(15_deg) AngleThreshold,
  }),
});

class CodeReleasePositionForKickOffCard : public CodeReleasePositionForKickOffCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return false;  // falta a;adir condicion.
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::codeReleasePositionForKickOff);
    theLookForwardSkill();
    //theStandSkill();
    // Not implemented in the Code Release.
    //theSaySkill("Please implement a behavior for me!");
    if(theRobotInfo.number == 1)
    {
      if((theRobotPose.translation - KeeperPos.translation).norm() > StopThreshold)
      {
        thePathToTargetSkill(0.7f, KeeperPos);
      }
      else if (theRobotPose.rotation < -AngleThreshold || theRobotPose.rotation > AngleThreshold)
      {
        theWalkAtRelativeSpeedSkill(Pose2f(1.0f, 0.f, 0.f));
      }else {
        theStandSkill();
      }
    }
    else if(theRobotInfo.number == 2)
    {
      if((theRobotPose.translation - Defender1Pos.translation).norm() > StopThreshold)
      {
        thePathToTargetSkill(0.6f, Defender1Pos);
      }
      else if (theRobotPose.rotation < -AngleThreshold || theRobotPose.rotation > AngleThreshold)
      {
        theWalkAtRelativeSpeedSkill(Pose2f(1.0f, 0.f, 0.f));
      }
      else 
      {
        theStandSkill();
      }
    }
    else if(theRobotInfo.number == 3)
    {
      if((theRobotPose.translation - Defender2Pos.translation).norm() > StopThreshold)
      {
        thePathToTargetSkill(0.6f, Defender2Pos);
      }
      else if (theRobotPose.rotation < -AngleThreshold || theRobotPose.rotation > AngleThreshold)
      {
        theWalkAtRelativeSpeedSkill(Pose2f(1.0f, 0.f, 0.f));
      }
      else 
      {
        theStandSkill();
      }
    }
    else if(theRobotInfo.number == 5)
    {
      if((theRobotPose.translation - Defender3Pos.translation).norm() > StopThreshold)
      {
        thePathToTargetSkill(0.6f, Defender3Pos);
      }
      else if (theRobotPose.rotation < -AngleThreshold || theRobotPose.rotation > AngleThreshold)
      {
        theWalkAtRelativeSpeedSkill(Pose2f(1.0f, 0.f, 0.f));
      }
      else 
      {
        theStandSkill();
      }
    }
    else if(theRobotInfo.number == 4)
    {
      if((theRobotPose.translation - StrikerPos.translation).norm() > StopThreshold)
      {
        thePathToTargetSkill(0.6f, StrikerPos);
      }
      else if (theRobotPose.rotation < -AngleThreshold || theRobotPose.rotation > AngleThreshold)
      {
        theWalkAtRelativeSpeedSkill(Pose2f(1.0f, 0.f, 0.f));
      }
      else 
      {
        theStandSkill();
      }
    }
    else
    {
      theStandSkill();
    }
  }
};

MAKE_CARD(CodeReleasePositionForKickOffCard);
