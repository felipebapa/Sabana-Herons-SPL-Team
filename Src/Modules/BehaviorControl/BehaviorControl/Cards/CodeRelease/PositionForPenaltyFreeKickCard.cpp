/**
 * @file PositionForPenaltyFreeKickCard.cpp
 *
 * Ir a las posiciones necesarias durante los 30s de ready.
 *
 * @author Santi and Jose
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

#include "Representations/Communication/RobotInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"

CARD(PositionForPenaltyFreeKickCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
  CALLS(PathToTarget),
  CALLS(WalkAtRelativeSpeed),
  
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(GameInfo),
  REQUIRES(FieldDimensions),
  DEFINES_PARAMETERS(
  {,
    (Pose2f)(Pose2f(0,-4100,0)) KeeperPos,
    (Pose2f)(Pose2f(0,-2500,0)) Defender1Pos,
    (Pose2f)(Pose2f(0,-2000,1500)) Defender2Pos,
    (Pose2f)(Pose2f(0,-2500,-1500)) Defender3Pos,
    (Pose2f)(Pose2f(0, 2800, 0)) StrikerPos,
    (int)(100) StopThreshold,
    (float)(15_deg) AngleThreshold,
  }),
});

class PositionForPenaltyFreeKickCard : public PositionForPenaltyFreeKickCardBase
{
  bool preconditions() const override
  {
    return (theGameInfo.gamePhase == GAME_PHASE_NORMAL && theGameInfo.state == STATE_READY) && theGameInfo.setPlay == SET_PLAY_PENALTY_KICK;
  }

  bool postconditions() const override
  {
    return (theGameInfo.gamePhase != GAME_PHASE_NORMAL && theGameInfo.state != STATE_READY) && theGameInfo.setPlay != SET_PLAY_PENALTY_KICK;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::PositionForPenaltyFreeKick);
    theLookForwardSkill();
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
      if((theRobotPose.translation - Defender2Pos.translation).norm() > StopThreshold)
      {
        thePathToTargetSkill(1.0, Defender2Pos);
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
        thePathToTargetSkill(1.0, Defender3Pos);
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
        thePathToTargetSkill(1.0, StrikerPos);
        theSaySkill("Going to penalty mark");
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

MAKE_CARD(PositionForPenaltyFreeKickCard);
