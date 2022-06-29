/**
 * @file PenaltyStriker.cpp
 *
 * Pruebas
 *
 * @author Jose and Santi
 * 
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/BallModel.h"
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

CARD(PenaltyKeeperCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(Say),
  CALLS(SpecialAction),
  CALLS(LookAtAngles),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(BallModel),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(1000) initialWaitTime,
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,
    (int) choice,
  }),
});

class PenaltyKeeperCard : public PenaltyKeeperCardBase
{
  bool preconditions() const override
  {
    return (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber) ;
  }

  bool postconditions() const override
  {
    return (theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT && theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber);
  }
  
  option
  {
      theActivitySkill(BehaviorStatus::PenaltyKeeper);
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
          }
      }    
      state(searchForBall)
      {
          transition
          {
              if(-100 > theBallModel.estimate.position.y() && theBallModel.estimate.velocity.x() < -90)
                goto rightDive;
              if(100 < theBallModel.estimate.position.y() && theBallModel.estimate.velocity.x() < -90)
                goto leftDive;
              if(-100 < theBallModel.estimate.position.y() && theBallModel.estimate.position.y() < 100 && theBallModel.estimate.velocity.x() < -90)
                goto goDown;
          }
          action
          {
              theSaySkill("Vengase");
              //theLookForwardSkill();
              //theSpecialActionSkill(SpecialActionRequest::standHigh);
              theSpecialActionSkill(SpecialActionRequest::sitDownNew);
              theLookAtAnglesSkill(theFieldBall.positionRelative.angle(),2);
          }
      }     
      state(rightDive)
      {
          transition
          {

          }  
          action
          {
              theSpecialActionSkill(SpecialActionRequest::rightDive);
          }

      }
      state(leftDive)
      {
          transition
          {

          }   
          action
          {
              theSpecialActionSkill(SpecialActionRequest::leftDive);
          }

      }
      state(goDown)
      {
          transition
          {
          
          }
          action
          {
              theSpecialActionSkill(SpecialActionRequest::preventBall);
          }

      }
    }  
    Angle calcAngleToGoal() const
    {
        return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
    } 


};

MAKE_CARD(PenaltyKeeperCard);