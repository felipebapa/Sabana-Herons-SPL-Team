/**
 * @file WaitForKickInCard.cpp
 *
 * Pruebas Wait para kick in.
 *
 * @author Santi
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

CARD(WaitForKickInCard,
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
    (Pose2f)(Pose2f(0,0,0)) KickInWaitPos,
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
  }),
});

class WaitForKickInCard : public WaitForKickInCardBase
{
  bool preconditions() const override
  {
    return (theGameInfo.setPlay == SET_PLAY_KICK_IN && theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber && theRobotInfo.number == 5/*#Robot a posicionarse.*/);
  }

  bool postconditions() const override
  {
    return (theGameInfo.setPlay != SET_PLAY_KICK_IN || theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber || theRobotInfo.number != 5/*#Robot a posicionarse.*/);
  }
  
  void execute() override
  {
      theActivitySkill(BehaviorStatus::WaitForKickIn);
      if((theRobotPose.translation.x() != 0) && (theRobotPose.translation.y() != 0)){
        thePathToTargetSkill(1.0, KickInWaitPos);
        theSaySkill("Going to center");
      }else{
        theStandSkill();
        theSaySkill("In position");
      }
  }
};

MAKE_CARD(WaitForKickInCard);
