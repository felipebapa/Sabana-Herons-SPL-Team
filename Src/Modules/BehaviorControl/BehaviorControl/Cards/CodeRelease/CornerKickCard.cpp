/**
 * @file CornerKickCard.cpp
 *
 * Pruebas Striker para tiro de esquina.
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

#include <string>

CARD(CornerKickCard,
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
    (int) choice,
  }),
});

class CornerKickCard : public CornerKickCardBase
{
  bool preconditions() const override
  {
    return ((theGameInfo.gamePhase == GAME_PHASE_NORMAL && theGameInfo.setPlay == SET_PLAY_CORNER_KICK) && theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber && theRobotInfo.number == 4);
  }

  bool postconditions() const override
  {
    return ((theGameInfo.gamePhase != GAME_PHASE_NORMAL && theGameInfo.setPlay != SET_PLAY_CORNER_KICK) || theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber || theRobotInfo.number != 4);
  }
  
  option
  {
      theActivitySkill(BehaviorStatus::CornerKick);
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
          goto walkToBall;
      }

      action
      {
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
        {
          if (choice==1){
            goto alignToPassTo5;   
          }
          if (choice==0){
            goto alignToPassTo3;
          }
        }
        //Aqui deberia decidir segun si el 5 o 3 pueden llegar a su posicion, por ahora
        //se hara un random choice.
      }

      action
      {
        srand((int)time(NULL));
        choice=0+rand()%(2-0);
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
      }
    }

    state(alignToPassTo5)
    {
      const Angle angleToPass = calcAngleToTeammate5();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToPass) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
          goto alignBehindBallFor5;
      }

      action
      {
        theLookForwardSkill();
        theSaySkill("Five");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToPass, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
      }
    }

    state(alignBehindBallFor5)
    {
      const Angle angleToPass = calcAngleToTeammate5();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToPass) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
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
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToPass, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }

    state(alignToPassTo3) 
    {
      const Angle angleToPass = calcAngleToTeammate3();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToPass) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
          goto alignBehindBallFor3;
      }

      action
      {
        theLookForwardSkill();
        theSaySkill("Three");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToPass, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
      }
    }
    
    state(alignBehindBallFor3)
    {
      const Angle angleToPass = calcAngleToTeammate3();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToPass) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
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
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToPass, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
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
  Angle calcAngleToTeammate5() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentPenaltyMark,0.f)).angle();
  }
  Angle calcAngleToTeammate3() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentPenaltyMark,500.f)).angle();
  }

};

MAKE_CARD(CornerKickCard);
