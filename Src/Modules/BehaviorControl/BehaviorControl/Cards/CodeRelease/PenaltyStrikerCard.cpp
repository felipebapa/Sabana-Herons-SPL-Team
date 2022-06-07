/**
 * @file PenaltyStriker.cpp
 *
 * Pruebas
 *
 * @author Jose Pinzon and Santi
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

CARD(PenaltyStrikerCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
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
    (int) (1) choice,
    (float) (0.3f) kickDist,
  }),
});

class PenaltyStrikerCard : public PenaltyStrikerCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT;
  }

  bool postconditions() const override
  {
    return theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT;
  }
  
  option
  {
      theActivitySkill(BehaviorStatus::PenaltyStriker);
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
      
      state(searchForBall)
      {
          transition
          {
              if(theFieldBall.ballWasSeen())
                  goto chooseSide;
          }
          action
          {
              
          }
      }
      
      state(chooseSide)
      {
          transition
          {
              if (choice==1){
                  goto alignLeft;
              }
              if (choice==0){
                  goto alignRight;
              }
          }
          action
          {
              srand((int)time(NULL));
              choice=0+rand()%(2-0);
          }

      }
      
      state(alignRight)
      {
          const Angle angleToGoal = calcAngleToGoal();

          transition
          {
              if(std::abs(angleToGoal) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
                  goto kick;
          }
          action
          {
              theWalkToTargetSkill(Pose2f(walkSpeed, 0.35f,0.35f), Pose2f(angleToGoal - 0.40f, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
          }
          
      }
       state(alignLeft)
      {
          const Angle angleToGoal = calcAngleToGoal();
          
          transition
          {
              if(std::abs(angleToGoal) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
                  goto kick;
          }
          action
          {
              theWalkToTargetSkill(Pose2f(walkSpeed, 0.35f,0.35f), Pose2f(angleToGoal + 0.40f, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
          }
          
      }
      state(kick)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theKickSkill.isDone()))
          goto start;
      }

      action
      {
        theLookForwardSkill();
        //theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        theKickSkill((KickRequest::kickForward), false, kickDist, false);
        
      }
    }
    state(stand)
    {
        transition
        {
            
        }
        action
        {
            theStandSkill();
        }
    }
    }
    
    Angle calcAngleToGoal() const
    {
        return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
    } 


};

MAKE_CARD(PenaltyStrikerCard);
