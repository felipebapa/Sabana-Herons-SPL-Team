/**
 * @file InitialCard.cpp
 *
 * This file specifies the behavior for a robot in the Penalty Game Phase.
 *
 * @author Jose P y Santi.
 */

#include "Representations/Communication/GameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/TeamCard.h"
#include "Representations/BehaviorControl/TeamSkills.h"
#include "Representations/Communication/TeamInfo.h"

TEAM_CARD(PenaltyShootoutTeamCard,
{,
  CALLS(Role),
  CALLS(TeamActivity),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
});

class PenaltyShootoutTeamCard : public PenaltyShootoutTeamCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT;
  }

  bool postconditions() const override
  {
    return theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT;
  }

  void execute() override
  {
    theTeamActivitySkill(TeamBehaviorStatus::PenaltyShootoutTeam);

    if (theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber){
      Role PenaltyStriker;
      PenaltyStriker.isGoalkeeper = false;
      PenaltyStriker.playBall = true;  
      theRoleSkill(PenaltyStriker);
      
    }else if(theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber){
      Role Goalie;
      Goalie.isGoalkeeper = true;
      Goalie.playBall = false;
      theRoleSkill(Goalie);
    }
  }
};

MAKE_TEAM_CARD(PenaltyShootoutTeamCard);
  
