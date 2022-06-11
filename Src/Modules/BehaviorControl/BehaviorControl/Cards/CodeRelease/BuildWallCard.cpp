/**
 * @file BuildWallCard.cpp
 *
 * Tests for defensive strategy
 *
 * @author Santi and Jose
 * 
 * 
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

CARD(BuildWallCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(Say),
  
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),


  
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(500) initialWaitTime,
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

class BuildWallCard : public BuildWallCardBase
{
  bool preconditions() const override
  {
    return ((theGameInfo.gamePhase == GAME_PHASE_NORMAL && theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK) && theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber && theRobotInfo.number == 1);
  }

  bool postconditions() const override
  {
    return ((theGameInfo.gamePhase != GAME_PHASE_NORMAL && theGameInfo.setPlay != SET_PLAY_PUSHING_FREE_KICK) || theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber || theRobotInfo.number != 1);
  }
  
  option
  {
      theActivitySkill(BehaviorStatus::BuildWall);
      initial_state(start)
      {
          transition
          {
             if(state_time > initialWaitTime)
               goto searchForBall;
          }
          action
          {
              theLookForwardSkill();
              theStandSkill();
              theSaySkill("Helllouuuu");
          }
      }
      
      state(turnToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold)
          goto walkToPos;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
      }
    }

    state(walkToPos)
    {
      const Angle angleOwnGoal = calcAngleOwnGoal();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
          goto buildWall;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleOwnGoal, theFieldBall.positionRelative.x() - 900.f, 0.0f));
      }
    }

    state(buildWall)
    {
      const Angle angleOwnGoal = calcAngleOwnGoal();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleOwnGoal, theFieldBall.positionRelative.x() - 900.f, 0.0f ));
      }
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
  }  
  
  Angle calcAngleOwnGoal() const
  {
    return(theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle();
  }
};

MAKE_CARD(BuildWallCard);
