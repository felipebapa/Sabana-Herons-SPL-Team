/**
 * @file Keeper.cpp
 *
 * Pruebas
 *
 * @author Andres Ramirez
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"

#include "Representations/Communication/RobotInfo.h"

CARD(KeeperCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(KeyFrameSingleArm),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(1000) initialWaitTime,
    (int)(3000) ballNotSeenTimeout,
    (Angle)(7_deg) ballAlignThreshold,
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
	(float)
  }),
});

class KeeperCard : public KeeperCardBase
{
  bool preconditions() const override
  {
    return theRobotInfo.number == 1;
  }

  bool postconditions() const override
  {
    return theRobotInfo.number != 1;
  }

  option
  {
    theActivitySkill(BehaviorStatus::Keeper);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto turnToBall;
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
		if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
          goto GoalRiskRight;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
		
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
	
	    state(GoalRiskRight)
    {
	  const Angle angleToGoal = calcAngleToGoal();
	  
      transition
      {
		if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
	    
      }

      action
      {
		  
		  theLookForwardSkill();
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
		  
		  //theWalkAtRelativeSpeedSkill(Pose2f(0.f, walkSpeed, 0.f));
		//theKeyFrameSingleArmSkill(ArmKeyFrameRequest::back, Arms::right, false);
      }
    }
  }
  
    Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }
  

};

MAKE_CARD(KeeperCard);
