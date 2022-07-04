/**
 * @file "GoalFreeKick".cpp
 *
 * Pruebas saque de arco.
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

#include <string>

CARD(GoalFreeKickCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(Say),
  CALLS(PathToTarget),
  
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

    (Pose2f)(Pose2f(0,-3000,1000)) Defender1Pos,
    (Pose2f)(Pose2f(0,-1500,2000)) Defender2Pos,
    (Pose2f)(Pose2f(0,-1500,-2000)) Defender3Pos,
    (Pose2f)(Pose2f(0,2000,0)) StrikerPos,
    (Pose2f)(Pose2f(0,-500.f,0)) KickInWaitPos,
    (int)(100) StopThreshold,
    (float)(15_deg) AngleThreshold,
  }),
});

class GoalFreeKickCard : public GoalFreeKickCardBase
{
  bool preconditions() const override
  {
    return ((theGameInfo.competitionPhase == GAME_PHASE_NORMAL && theGameInfo.setPlay == SET_PLAY_GOAL_KICK) && theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber);
  }

  bool postconditions() const override
  {
    return ((theGameInfo.competitionPhase != GAME_PHASE_NORMAL && theGameInfo.setPlay != SET_PLAY_GOAL_KICK) && theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber);
  }
  
  option
  {
      theActivitySkill(BehaviorStatus::GoalFreeKick);
      initial_state(start)
      {
        
          transition
          {
             if(state_time > initialWaitTime && theRobotInfo.number==1)
               goto searchForBall;
             if(theRobotInfo.number!=1)
              goto positions;
          }
          action
          {
              theLookForwardSkill();
              theStandSkill();
              theSaySkill("Clear!");
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
        theSaySkill("T");
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
      }

      action
      {
        theSaySkill("W");
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
          goto kick;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }

    state(kick)
    {
      //const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theKickSkill.isDone()))
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theKickSkill((KickRequest::kickForward), true, 0.3f, false);
        theSaySkill("Go Go");
      }
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
      }

      action
      {
        theLookForwardSkill();
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
        if(theRobotInfo.number == 2)
    {
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
    }
    else if(theRobotInfo.number == 3)
    {
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
    }
    else if(theRobotInfo.number == 5)
    {
      if((theRobotPose.translation.x() != -500.f) && (theRobotPose.translation.y() != 0)){
        thePathToTargetSkill(1.0, KickInWaitPos);
        theSaySkill("Pass!");
      }else{
        theStandSkill();
      
        theSaySkill("In position");
      }
    }
    else if(theRobotInfo.number == 4)
    {
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
    }
    else
    {
      theStandSkill();
    } 
  
      }
    }
  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }  

};

MAKE_CARD(GoalFreeKickCard);
