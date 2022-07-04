/**
 * @file Kick in.cpp
 *
 * Pruebas Striker para kick in.
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
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"


#include <string>

CARD(KickInCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(LookAtAngles),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(InWalkKick),
  CALLS(Kick),
  CALLS(Say),
  CALLS(PathToTarget),
  
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(TeamBallModel),
  REQUIRES(LibCheck),
  
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
    
    (Pose2f)(Pose2f(0,-4450,0)) KeeperPos,
    (Pose2f)(Pose2f(0,-3000,0)) Defender1Pos,
    (Pose2f)(Pose2f(0,-2500,1500)) Defender2Pos,
    (Pose2f)(Pose2f(0,-2000,0)) StrikerPos,
    (Pose2f)(Pose2f(0,0,0)) KickInWaitPos,
    (int)(100) StopThreshold,
    (float)(15_deg) AngleThreshold,
  }),
});

class KickInCard : public KickInCardBase
{
  bool preconditions() const override
  {
    return ((theGameInfo.gamePhase == GAME_PHASE_NORMAL && theGameInfo.setPlay == SET_PLAY_KICK_IN) && theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber);
  }

  bool postconditions() const override
  {
    return ((theGameInfo.gamePhase != GAME_PHASE_NORMAL && theGameInfo.setPlay != SET_PLAY_KICK_IN && theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber));
  }
  
  option
  {
    theActivitySkill(BehaviorStatus::KickIn);
    initial_state(start)
      {
          transition
          {
             if(theRobotInfo.number == 1)
              goto positions;
             if(state_time > initialWaitTime)
               goto searchForBallInit;
          }
          action
          {
              theLookForwardSkill();
              theStandSkill();
              theSaySkill("Kick In Card");
              
          }
      }    
    state(searchForBallInit)
    {
      transition
      {
        if(state_time > 1500)
          goto lookLeftInit;
        if(theFieldBall.ballWasSeen() || theTeamBallModel.isValid)
          goto check;
        
      }

      action
      {
        //theLookForwardSkill();
        theLookAtAnglesSkill(-1,2, 0.6f);
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
    state(lookLeftInit)
    {
      transition
      {
        if(state_time > 1500)
          goto searchForBallInit;
        if(theFieldBall.ballWasSeen() || theTeamBallModel.isValid)
          goto check;
      }
      action
      {
        //theLookForwardSkill();
        theLookAtAnglesSkill(1,2,0.6f);
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
    state(check)
    {
      transition
      {
        if(theTeamBallModel.isValid)
        {
          theSaySkill("Valid");
          if(theFieldBall.ballWasSeen())
            goto turnToBall;
          else
          {
            theSaySkill("Got to Pos");
            goto positions;
          }
        }
      }
      action
      {
        theLookAtAnglesSkill(theFieldBall.positionRelative.angle() , 2);
      }
    }
    state(turnToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold)
          goto walkToBall;
      }

      action
      {
        theSaySkill("Turning");
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
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
        if(theLibCheck.closerToTheBall != theRobotInfo.number)
        {
          theSaySkill("No longer Closest");
          goto positions;    
        }
      }
      action
      {
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
      transition
      {
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
      }
      action
      {
        theLookForwardSkill();
        theKickSkill((KickRequest::kickForward), true,0.2f, false);
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

      }
      action
      {
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
            theSaySkill("Waiting");
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
            theSaySkill("Waiting");
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
          if((theRobotPose.translation.x() != 0) && (theRobotPose.translation.y() != 0)){
            thePathToTargetSkill(1.0, KickInWaitPos);
            theSaySkill("Going to center");
          }else{
            theStandSkill();
            theSaySkill("In position");
          }
        }
        else if(theRobotInfo.number == 4)
        {
          if((theRobotPose.translation.x() != 0) && (theRobotPose.translation.y() != 0)){
            thePathToTargetSkill(1.0, StrikerPos);
            theSaySkill("Waiting");
          }else{
            theStandSkill();
            theSaySkill("In position");
          }
        }
      }
    }
  }
  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }  
};

MAKE_CARD(KickInCard);
