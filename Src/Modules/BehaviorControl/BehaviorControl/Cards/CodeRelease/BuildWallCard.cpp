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
  CALLS(LookAtAngles),
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
    (Angle)(180_deg) OwnGoalOffset,

    (Pose2f)(Pose2f(0,-4050,0)) KeeperPos,
    (Pose2f)(Pose2f(0,-3000,0)) Defender1Pos,
    (Pose2f)(Pose2f(0,-2500,1500)) Defender2Pos,
    (Pose2f)(Pose2f(0,-2500,-1500)) Defender3Pos,
    (Pose2f)(Pose2f(0,-1000,0)) StrikerPos,
    (int)(100) StopThreshold,
    (float)(15_deg) AngleThreshold,
  }),
});

class BuildWallCard : public BuildWallCardBase
{
  bool preconditions() const override
  {
    return ((theGameInfo.gamePhase == GAME_PHASE_NORMAL && (theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK || theGameInfo.setPlay == SET_PLAY_KICK_IN || theGameInfo.setPlay == SET_PLAY_GOAL_KICK || theGameInfo.setPlay == SET_PLAY_CORNER_KICK )) && (theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber));
  }

  bool postconditions() const override
  {
    return ((theGameInfo.gamePhase != GAME_PHASE_NORMAL && theGameInfo.setPlay != SET_PLAY_PUSHING_FREE_KICK && theGameInfo.setPlay != SET_PLAY_KICK_IN && theGameInfo.setPlay != SET_PLAY_GOAL_KICK && theGameInfo.setPlay != SET_PLAY_CORNER_KICK  ) || (theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber));
  }
  
  option
  {
    theActivitySkill(BehaviorStatus::BuildWall);
    initial_state(start)
      {
          transition
          {
             if(theRobotInfo.number == 1)
               goto positions;
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
    state(exclusion)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;

        if(theFieldBall.positionRelative.norm() < 1000.f && theRobotInfo.number != 4){
          goto walkToPos;
        }else if((theRobotInfo.number == 4 && theFieldBall.positionRelative.norm() < 100.f) && theFieldBall.positionOnField.x() >= 0){
          goto walkToPos;
        }else{
          goto positions;
        }
      }

      action
      {
        theLookForwardSkill();
      }
    }
    state(turnToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold)
          goto exclusion;
      }

      action
      {
        theLookForwardSkill();
        theSaySkill("TURN TO BALL");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
      }
    }
    state(walkToPos)
    {
      const Angle angleOwnGoal = calcAngleOwnGoal();
      const Angle angleBall = calcAngleBall();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(theFieldBall.positionRelative.norm() > 800.f && (angleBall < angleOwnGoal - 5_deg || angleBall > angleOwnGoal + 5_deg))
          goto buildWall;
      }

      action
      {
        theLookAtAnglesSkill(theFieldBall.positionRelative.angle(),2);
        theSaySkill("Walk to Ball");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleOwnGoal + OwnGoalOffset , theFieldBall.positionRelative.x() - 900.f ,theFieldBall.positionRelative.y()));
      }
    }
    state(buildWall)
    {
      //const Angle angleOwnGoal = calcAngleOwnGoal();

      transition
      {
        if(!theFieldBall.ballWasSeen(10000))
          goto searchForBall;
        if(theFieldBall.positionRelative.norm() <= 800.f )
          goto walkToPos;
      }

      action
      {
        theLookForwardSkill();
        theSaySkill("COMPLETE");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), theFieldBall.positionRelative.x() - 800.f, 0.0f ));
      }
    }
    state(searchForBall)
    {
      transition
      {
        if(state_time >1500)
          goto lookLeft;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(!theFieldBall.ballWasSeen(15000))
          goto positions;
      }

      action
      {
        theLookForwardSkill();
        theLookAtAnglesSkill(-1,2);
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
    state(lookLeft)
    {
      transition
      {
        if(state_time > 1500)
          goto searchForBall;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(!theFieldBall.ballWasSeen(15000))
          goto positions;
      }
      action
      {
        theLookForwardSkill();
        theLookAtAnglesSkill(1,2);
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
    state(positions)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
      }

      action
      {
        theLookForwardSkill();
        if(theRobotInfo.number == 1){
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
        }else if(theRobotInfo.number == 2){
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
        }else if(theRobotInfo.number == 3){
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
        }else if(theRobotInfo.number == 5){
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
        }else if(theRobotInfo.number == 4){
          if((theRobotPose.translation - StrikerPos.translation).norm() > StopThreshold)
          {
            thePathToTargetSkill(1.0, StrikerPos);
          }
          else if (theRobotPose.rotation < -AngleThreshold || theRobotPose.rotation > AngleThreshold)
          {
            theWalkAtRelativeSpeedSkill(Pose2f(1.0f, 0.f, 0.f));
          }
          else 
          {
            theStandSkill();
          }
        }else{
          theStandSkill();
        }
      }
    }
  }   
  Angle calcAngleOwnGoal() const
  {
    return(theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle();
  }
  Angle calcAngleBall() const
  {
    return(Vector2f(theFieldBall.positionOnField.x(), theFieldBall.positionOnField.y()) + Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle();
  }
};

MAKE_CARD(BuildWallCard);
