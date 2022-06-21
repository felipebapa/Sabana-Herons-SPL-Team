/**
 * @file PushingFreekickCard.cpp
 *
 * Tests for Pushingfreekick
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
#include "Representations/Modeling/ObstacleModel.h"

#include <string>

CARD(PushingFreekickCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(LookAtAngles),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(Say),
  CALLS(PathToTarget),
  CALLS(InWalkKick),
  
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(ObstacleModel),
  
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
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

    (Pose2f)(Pose2f(0,-4050,0)) KeeperPos,
    (Pose2f)(Pose2f(0,-3000,0)) Defender1Pos,
    (Pose2f)(Pose2f(0,-2500,1500)) Defender2Pos,
    (Pose2f)(Pose2f(0,2500,-1500)) Defender3Pos,
    (Pose2f)(Pose2f(0,1000,0)) StrikerPos,
    (int)(100) StopThreshold,
    (float)(15_deg) AngleThreshold,
  }),
});

class PushingFreekickCard : public PushingFreekickCardBase
{
  bool preconditions() const override
  {
    return ((theGameInfo.gamePhase == GAME_PHASE_NORMAL && theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK) && theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber);
  }

  bool postconditions() const override
  {
    return ((theGameInfo.gamePhase != GAME_PHASE_NORMAL && theGameInfo.setPlay != SET_PLAY_PUSHING_FREE_KICK) || theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber);
  }
  
  option
  {
    theActivitySkill(BehaviorStatus::PushingFreekick);
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
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
      }
    }

    state(exclusion)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;

        if(theFieldBall.positionRelative.norm() < 1000.f && theRobotInfo.number != 4){
          goto walkToBall;
        }else if((theRobotInfo.number == 4 && theFieldBall.positionRelative.norm() < 3000.f)){
          goto walkToBallStriker;
        }else{
          goto positions;
        }
      }

      action
      {
        theLookForwardSkill();
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

    state(walkToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
          goto alignToGoal;
      }

      action
      {
        theSaySkill("Not Striker");
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
      }
    }

    state(walkToBallStriker)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
          goto alignToGoal;
      }

      action
      {
        theSaySkill("Striker");
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
      }
    }

    state(alignToGoal)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
          goto alignBehindBall;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
      }
    }

    state(alignBehindBall)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToGoal) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
          {
          if(!theObstacleModel.obstacles.empty()){     //Tenemos obstàculos, entonces, actuamos.   
            for(const auto& obstacle : theObstacleModel.obstacles){
              if ((obstacle.center.x() < (theFieldDimensions.xPosOpponentGoal - theFieldBall.positionOnField.x())))
                goto kick;
              if (std::abs(obstacle.center.y()) > 150.f)
                goto longKick;
              }
              }
              else
                goto longKick;    
          }
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }

    state(kick)
    {
      const Angle angleToGoal = calcAngleToGoal();
      
      transition
      {
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        
      }
    }

    state(longKick)
    {     
      transition
      {
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theKickSkill((KickRequest::kickForward), true, 0.3f, false);
      }
    }

    state(searchForBall)
    {
      transition
      {
        if(state_time > 1500)
          goto lookLeft;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
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
      }
      action
      {
        theLookForwardSkill();
        theLookAtAnglesSkill(1,2);
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
  }
  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }
};



MAKE_CARD(PushingFreekickCard);
