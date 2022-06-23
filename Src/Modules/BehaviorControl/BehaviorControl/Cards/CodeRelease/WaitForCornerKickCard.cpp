/**
 * @file WaitForCornerKickCard.cpp
 *
 * Pruebas Wait para tiro de esquina.
 *
 * @author Santi and Jose
 * 
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamInfo.h"

#include <string>

CARD(WaitForCornerKickCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(Say),
  CALLS(PathToTarget),
  
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (Pose2f)(Pose2f(0,0,0)) CornerKickWaitPos,
    (int)(1000) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
    (Angle)(5_deg) ballAlignThreshold,
    (float)(500.f) ballNearThreshold,
    (Angle)(10_deg) angleToGoalThreshold,
    (float)(400.f) ballAlignOffsetX,
    (float)(100.f) ballYThreshold,
    (Angle)(2_deg) angleToGoalThresholdPrecise,
    (float)(150.f) ballOffsetX,
    (Rangef)({140.f, 170.f}) ballOffsetXRange,
    (float)(40.f) ballOffsetY,
    (Rangef)({20.f, 50.f}) ballOffsetYRange,
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,

    (Pose2f)(Pose2f(0,-4450,0)) KeeperPos,
    (Pose2f)(Pose2f(0,-3000,0)) Defender1Pos,
    (Pose2f)(Pose2f(0,-2500,1500)) Defender2Pos,
    (Pose2f)(Pose2f(0,-2000,0)) StrikerPos,
    (int)(100) StopThreshold,
    (float)(15_deg) AngleThreshold,
  }),
});

class WaitForCornerKickCard : public WaitForCornerKickCardBase
{
  bool preconditions() const override
  {
    return (theGameInfo.setPlay == SET_PLAY_CORNER_KICK && theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber);
  }

  bool postconditions() const override
  {
    return (theGameInfo.setPlay != SET_PLAY_CORNER_KICK || theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber);
  }
  
  void execute() override
  {
    theActivitySkill(BehaviorStatus::WaitForCornerKick);

    if(theRobotInfo.number == 1)
    {
      if((theRobotPose.translation - KeeperPos.translation).norm() > StopThreshold)
      {
        thePathToTargetSkill(1.0, KeeperPos);
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
    else if(theRobotInfo.number == 2)
    {
      if((theRobotPose.translation - Defender1Pos.translation).norm() > StopThreshold)
      {
        thePathToTargetSkill(1.0, Defender1Pos);
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
      if((theRobotPose.translation.x() != theFieldDimensions.xPosOpponentPenaltyMark-500.f) && (theRobotPose.translation.y() != theFieldDimensions.yPosCenterGoal+1000.f)){
        thePathToTargetSkill(1.0, Pose2f(0,theFieldDimensions.xPosOpponentPenaltyMark-500.f,theFieldDimensions.yPosCenterGoal+1000.f));
        theSaySkill("Going to recieve pass");
      }else{
        theStandSkill();
        theSaySkill("In position");
      }
    }
    else if(theRobotInfo.number == 5)
    {
      if((theRobotPose.translation.x() != theFieldDimensions.xPosOpponentPenaltyMark-200.f) && (theRobotPose.translation.y() != theFieldDimensions.yPosCenterGoal-1000.f)){
        thePathToTargetSkill(1.0, Pose2f(0,theFieldDimensions.xPosOpponentPenaltyMark-200.f,theFieldDimensions.yPosCenterGoal-1000.f));
        theSaySkill("Going to recieve pass");
      }else{
        theStandSkill();
        theSaySkill("In position");
      }
    }
  }
};

MAKE_CARD(WaitForCornerKickCard);