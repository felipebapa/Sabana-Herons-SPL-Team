/**
 * @file InitialCard.cpp
 *
 * Prueba carta de equipo para manejo de penalty. 
 * Esta puede que no sea necesaria segun lo que hemos probado. Tal vez mas adelante en pruebas veamos que si es necesaria.
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
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  void execute() override
  {
    theTeamActivitySkill(TeamBehaviorStatus::PenaltyShootoutTeam);

    /*if (theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber){
      Role PenaltyStriker;
      PenaltyStriker.isGoalkeeper = false;
      PenaltyStriker.playBall = true;  
      theRoleSkill(PenaltyStriker);
      
    }else if(theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber){
      Role Goalie;
      Goalie.isGoalkeeper = true;
      Goalie.playBall = false;
      theRoleSkill(Goalie);
    }*/
  }
};

MAKE_TEAM_CARD(PenaltyShootoutTeamCard);
  
