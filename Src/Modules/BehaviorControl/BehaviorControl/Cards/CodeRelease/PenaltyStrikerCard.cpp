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

#include <string>

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
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,
    (int) choice,
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
              theSaySkill("Searching");
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
              //choice=0;
          }

      }
      
      state(alignRight)
      {
          const Angle angleToGoal = calcAngleToGoal();

          transition
          {
              if(std::abs(theFieldBall.positionRelative.x()) < 200.f && std::abs(theFieldBall.positionRelative.y())  < 50.f)
                  goto kick;
          }
          action
          {
              theWalkToTargetSkill(Pose2f(walkSpeed, 0.35f,0.35f), Pose2f(angleToGoal - 0.4f, theFieldBall.positionRelative.x() - 150.f, theFieldBall.positionRelative.y() + 50.f));
              theSaySkill("Right Right Right");
          }
          
      }
       state(alignLeft)
      {
          const Angle angleToGoal = calcAngleToGoal();
          
          transition
          {
              if(std::abs(theFieldBall.positionRelative.x()) < 200.f && std::abs(theFieldBall.positionRelative.y())  < 100.f)
                  goto kick;
          }
          action
          {
              theWalkToTargetSkill(Pose2f(walkSpeed, 0.35f,0.35f), Pose2f(angleToGoal + 0.4f, theFieldBall.positionRelative.x() - 150.f, theFieldBall.positionRelative.y() + 50.f));
              theSaySkill("Left Left Left");
          }
          
      }
      state(kick)
    {
      //const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theKickSkill.isDone()))
          goto stand;
      }

      action
      {
        theLookForwardSkill();
        //theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        theKickSkill((KickRequest::kickForward), false, 0.3f, false);
        
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
