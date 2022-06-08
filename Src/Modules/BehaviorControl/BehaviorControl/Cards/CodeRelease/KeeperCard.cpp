/**
 * @file Keeper.cpp
 *
 * Pruebas
 *
 * @author Felipe Barreto
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/BallModel.h"
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
  CALLS(PathToTarget),
  CALLS(KeyFrameSingleArm),
  CALLS(SpecialAction),
  CALLS(LookAtPoint),
  REQUIRES(FieldBall),
  REQUIRES(BallModel),
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
	(float)(100.f) ballXThreshold,
    (Angle)(2_deg) angleToGoalThresholdPrecise,
    (float)(150.f) ballOffsetX,
    (Rangef)({140.f, 170.f}) ballOffsetXRange,
    (float)(40.f) ballOffsetY,
    (Rangef)({20.f, 50.f}) ballOffsetYRange,
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,
	(Pose2f)(Pose2f(0,-4450,0)) KeeperPos,
    (int)(500) StopThreshold,
    (float)(15_deg) AngleThreshold,
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
          goto coverBallTrajectory;
      }

      action
      {
        theLookForwardSkill();
		//theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(),theFieldBall.positionRelative.y(),0.f),(HeadMotionRequest::targetOnGroundMode));
        theStandSkill();
		
      }
    }
	
	state(coverBallTrajectory)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
		    //if(-100 > theBallModel.estimate.position.y() && theBallModel.estimate.velocity.x() < -90)
		    if(-100 > theBallModel.estimate.position.y() && theFieldBall.endPositionRelative.x() < ballXThreshold)
          goto GoalRiskRight;
		    //if( 100 < theBallModel.estimate.position.y() && theBallModel.estimate.velocity.x() < -90)
		    if( 100 < theBallModel.estimate.position.y() && theFieldBall.endPositionRelative.x() < ballXThreshold)
          goto GoalRiskLeft;
        if(theFieldBall.positionRelative.norm() >= 2500.0f)
          goto guardGoal;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));

		    if(theBallModel.estimate.position.x() < 3500){

           //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f)); //No quiero que camine a este angulo.
		      //Quiero que camine horizontalmente hasta que el angulo sea 0.

		   theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldBall.positionRelative.y()));  //Pose2f (angulo,x,y)


		    }

      }
    }
	
    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen() && theBallModel.estimate.position.x() < 3500)
          goto coverBallTrajectory;
        if(!theFieldBall.ballWasSeen(10000))
          goto goBackHome;   
      }

      action
      {
        theLookForwardSkill();
		
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
	
	    state(GoalRiskRight)
    {
	  //const Angle angleToGoal = calcAngleToGoal();
	  
      transition
      {
		    if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if((theRobotPose.translation.x() > theFieldDimensions.xPosOwnGroundline) || (theRobotPose.translation.y() > theFieldDimensions.yPosLeftGoal) || (theRobotPose.translation.y() < theFieldDimensions.yPosRightGoal))
          goto coverBallTrajectory;
	    
      }

      action
      {
		  
		  theLookForwardSkill();
          //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
		  theSpecialActionSkill(SpecialActionRequest::rightDive);
		  //theWalkAtRelativeSpeedSkill(Pose2f(0.f, walkSpeed, 0.f));
		//theKeyFrameSingleArmSkill(ArmKeyFrameRequest::back, Arms::right, false);
      }
    }
	
	   state(GoalRiskLeft)
    {
	  //const Angle angleToGoal = calcAngleToGoal();
	  
      transition
      {
		    if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
	    
      }

      action
      {
		  
		  theLookForwardSkill();
          //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
		  theSpecialActionSkill(SpecialActionRequest::leftDive);
		  //theWalkAtRelativeSpeedSkill(Pose2f(0.f, walkSpeed, 0.f));
		//theKeyFrameSingleArmSkill(ArmKeyFrameRequest::back, Arms::right, false);
      }
    }
	
    state(guardGoal) {
      transition 
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(theFieldBall.positionRelative.norm() < 2500.0f)
          goto coverBallTrajectory;
      }
      action
      {
        theLookForwardSkill();
        thePathToTargetSkill(walkSpeed, Pose2f(theFieldBall.positionRelative.angle(), theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosCenterGoal));
      }
    }

    state(goBackHome) {
      transition
      {
        if(theFieldBall.ballWasSeen() && theBallModel.estimate.position.x() < 3500)
          goto coverBallTrajectory;
      }
      action
      {
        theLookForwardSkill();
        thePathToTargetSkill(1.0, KeeperPos);
      }
    }

  }
  
    Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }
  

};

MAKE_CARD(KeeperCard);