/**
 * @file Keeper.cpp
 *
 * Pruebas
 *
 * @author DMF
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
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
  CALLS(LookAtAngles),
  CALLS(Say),
  REQUIRES(FieldBall),
  REQUIRES(BallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  REQUIRES(LibCheck),
  REQUIRES(ObstacleModel),
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

    bool hayObstaculoCerca = false;
      if(!theObstacleModel.obstacles.empty()){     //Tenemos obstàculos, entonces, actuamos.   
      for(const auto& obstacle : theObstacleModel.obstacles){
        //See if the obstacle is first than the target   
      if (obstacle.center.norm()<400.f)  
          hayObstaculoCerca=true;
      }
    }

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
      const Angle angleToGoal = calcAngleToGoal();
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
		    //if(-100 > theBallModel.estimate.position.y() && theBallModel.estimate.velocity.x() < -90)
        if(theRobotPose.translation.y() >= theFieldDimensions.yPosLeftGoal || theRobotPose.translation.y() <= theFieldDimensions.yPosRightGoal)
          goto waitBall;
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
		    if(theBallModel.estimate.position.x() < 3500)
		      theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, 0.f, theFieldBall.positionRelative.y()));  //Pose2f (angulo,x,y)
        if(theRobotPose.translation.y() > theFieldDimensions.yPosLeftGoal)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() - 200));  
        if(theRobotPose.translation.y() < theFieldDimensions.yPosRightGoal)  
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() + 200));  
      }
    }
	
	    state(GoalRiskRight)
    {
	  //const Angle angleToGoal = calcAngleToGoal();
	  
      transition
      {
		    if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
      }

      action
      {
		  
		  theLookForwardSkill();
          //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
		  //theSpecialActionSkill(SpecialActionRequest::rightDive);
      theSaySkill("Right Siuuuuuu");
		  //theWalkAtRelativeSpeedSkill(Pose2f(0.f, walkSpeed, 0.f));
      }
    }
    state(waitBall)
    {

      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(!theFieldBall.ballWasSeen(4000))
          goto goBackHome;
        if(hayObstaculoCerca)
          goto ObsAvoid;   
      }

      action
      {
          theLookForwardSkill();
          if(theRobotPose.translation.y() > theFieldDimensions.yPosLeftGoal)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() - 200));
          if(theRobotPose.translation.y() < theFieldDimensions.yPosRightGoal)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() + 200));
      }
    }
	   state(GoalRiskLeft)
    {
	  //const Angle angleToGoal = calcAngleToGoal();
	  
      transition
      {
		    if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
	    
      }

      action
      {
		  
		  theLookForwardSkill();
          //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
		  //theSpecialActionSkill(SpecialActionRequest::leftDive);
      theSaySkill("Left Siuuuuuu");
		  //theWalkAtRelativeSpeedSkill(Pose2f(0.f, walkSpeed, 0.f));
		//theKeyFrameSingleArmSkill(ArmKeyFrameRequest::back, Arms::right, false);
      }
    }
	
    state(guardGoal) {
      transition 
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(theFieldBall.positionRelative.norm() < 2500.0f)
          goto coverBallTrajectory;
        if(hayObstaculoCerca)
          goto ObsAvoid;
      }
      action
      {
        theLookForwardSkill();
        thePathToTargetSkill(walkSpeed, Pose2f(theFieldBall.positionRelative.angle(), theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosCenterGoal));
      }
    }

    state(turnToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(theBallModel.estimate.position.y() < theFieldDimensions.yPosLeftGoal)
          goto coverBallTrajectory;
        if(hayObstaculoCerca)
          goto ObsAvoid;
      }  
      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
        if(theRobotPose.translation.y() > theFieldDimensions.yPosLeftGoal)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() - 200));
        if(theRobotPose.translation.y() < theFieldDimensions.yPosRightGoal)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() + 200));
      }
    }  

    state(goBackHome) {
      transition
      {
        if(theFieldBall.ballWasSeen() && theBallModel.estimate.position.x() < 3500)
          goto coverBallTrajectory;
        if(hayObstaculoCerca)
          goto ObsAvoid;
      }
      action
      {
        theLookForwardSkill();
        thePathToTargetSkill(1.0, KeeperPos);
      }
    }

    state(GiraCabezaDer)
    {
      
      transition
      {
        if(theFieldBall.ballWasSeen() && theBallModel.estimate.position.x() < 3500)
          goto coverBallTrajectory;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout) && state_time > 1500)
          goto GiraCabezaIzq;
        if(hayObstaculoCerca)
          goto ObsAvoid;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;   
      }

      action
      {
        if(theRobotPose.translation.y() > theFieldDimensions.yPosLeftGoal)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() - 200));
          if(theRobotPose.translation.y() < theFieldDimensions.yPosRightGoal)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() + 200));
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
        theLookAtAnglesSkill(-1,2);
      }
    }

    state(GiraCabezaIzq)
    {
      transition
      {
        if(theFieldBall.ballWasSeen() && theBallModel.estimate.position.x() < 3500)
          goto coverBallTrajectory;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout) && state_time > 1500)
          goto GiraCabezaDer;
        if(hayObstaculoCerca)
          goto ObsAvoid;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
      }
      action
      {
          if(theRobotPose.translation.y() > theFieldDimensions.yPosLeftGoal)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() - 200));
          if(theRobotPose.translation.y() < theFieldDimensions.yPosRightGoal)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() + 200));
          theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
          theLookAtAnglesSkill(1,2);
          //a+=1;   
      }
    }

    state(ObsAvoid)
    {  
      transition 
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
      }
      action
      {
        for(const auto& obstacle : theObstacleModel.obstacles) {
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, obstacle.center.norm()+100)); 
        }
        if(theRobotPose.translation.y() > theFieldDimensions.yPosRightGoal)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosRightGoal+500));
      }
    }

  }
  
    Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }
  

};

MAKE_CARD(KeeperCard);