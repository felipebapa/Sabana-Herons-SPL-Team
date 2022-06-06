/**
 * @file InitialCard.cpp
 *
 * This file specifies the behavior for a robot in the Penalty Game Phase.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(PenaltyShootoutTeamCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  REQUIRES(GameInfo),
});

class PenaltyShootoutTeamCard : public PenaltyShootoutTeamCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  void execute() override
  {
      theActivitySkill(BehaviorStatus::PenaltyShootoutTeamCard);
      theLookForwardSkill();
      //theKickSkill((KickRequest::kickForward), false, 3.f, false);
  }
};

MAKE_CARD(PenaltyShootoutTeamCard);
  
