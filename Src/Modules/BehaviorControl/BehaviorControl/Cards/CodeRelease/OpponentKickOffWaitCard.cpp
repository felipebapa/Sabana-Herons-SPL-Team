/**
 * @file OpponentKickOffWaitCard.cpp
 *
 * Test para que esperemos a que saquen en el kick off el oponente.
 *
 * @author Santi
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Representations/Communication/RobotInfo.h"
#include "Tools/Modeling/Obstacle.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Modeling/BallModel.h"

#include  <iostream>


CARD(OpponentKickOffWaitCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(Say),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(OwnTeamInfo),
  REQUIRES(GameInfo),
  REQUIRES(BallModel),
  REQUIRES(LibCheck),
  
  DEFINES_PARAMETERS(
  {,
    (int)(11000)kickofftime,
    (float)(0.f)initialballpos,
    (bool)(false)exit,
  }),
});

class OpponentKickOffWaitCard : public OpponentKickOffWaitCardBase
{
  bool preconditions() const override
  {
    return false;
  }

  bool postconditions() const override
  {
    return exit;
  }

option
  {
    theActivitySkill(BehaviorStatus::OpponentKickOffWait);
    
    initial_state(start)
    {
      transition
      {
        if(theGameInfo.state == STATE_PLAYING){
          if(theRobotPose.translation.norm()>theFieldDimensions.centerCircleRadius){
            goto waitforkickoff;
          }
        }
      }
      action
      {
        theStandSkill();

        theSaySkill("Opponent kick off");
      }   
    }

    state(waitforkickoff)
    {
      transition
      {
        if( theLibCheck.TeammateSeeingBall == true  || state_time > kickofftime){
          exit = true;
          theSaySkill("True");
        }
      }
      action
      {
        theSaySkill("waiting");
      }

    }
  }
};
MAKE_CARD(OpponentKickOffWaitCard);
