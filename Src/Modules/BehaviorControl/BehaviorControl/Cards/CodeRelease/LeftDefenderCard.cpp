/**
 * @file LeftDefenderCard.cpp
 *
 * This file implements a basic striker behavior for the code release.
 * Normally, this would be decomposed into at least
 * - a ball search behavior card
 * - a skill for getting behind the ball
 *
 * @author Dap y Mia
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/ArmKeyFrameRequest.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
// #include "LookAroundCard.cpp"


CARD(LeftDefenderCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookAtAngles),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(KeyFrameArms),
  CALLS(PathToTarget),
  CALLS(SpecialAction),
  CALLS(Say),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(RobotInfo),
  REQUIRES(LibCheck),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(500) initialWaitTime,
    (int)(4000) ballNotSeenTimeout,
    (Pose2f)(Pose2f(0,-2500,1500)) Defender1Pos,
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
    (bool) direction,
    (int)(0) prueba,
  }),
});

class LeftDefenderCard : public LeftDefenderCardBase
{
  bool preconditions() const override
  {
    return theRobotInfo.number == 3;
  }

  bool postconditions() const override
  {
    return theRobotInfo.number != 3;
  }

  option
  {
    theActivitySkill(BehaviorStatus::LeftDefender);

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

    state(turnToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold)
          goto DefendBall;
        // if(theFieldBall.positionRelative.y() > theFieldDimensions.yPosLeftGoal)
        //   goto walkToBall;
		if(theLibCheck.closerToTheBall && theFieldBall.ballWasSeen(1000))  //El que estè viendo al balon y este mas cerca.
		  goto prueba;
      }  
      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
        if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));
      }
    }  

    state(walkToBall)
    {
      transition
      {
        if(theRobotPose.translation.y() <= theFieldDimensions.yPosLeftGoal)
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
          goto alignToGoal;
		if(theLibCheck.closerToTheBall && theFieldBall.ballWasSeen(1000))  //El que estè viendo al balon y este mas cerca.
		  goto prueba;
          
      }

      action
      {
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
      }
    }

    state(waitBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;

        //if(!theFieldBall.ballWasSeen(10000))
         // goto goBackHome;   
		if(theLibCheck.closerToTheBall && theFieldBall.ballWasSeen(1000))  //El que estè viendo al balon y este mas cerca.
		  goto prueba;

        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto lookLeft;
        // if(!theFieldBall.ballWasSeen(10000))
        //   goto goBackHome;   

      }

      action
      {
          theLookForwardSkill();
          theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
          if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));
      }
    }

    state(DefendBall)
    {
      transition
      {
        if(theRobotPose.translation.y() <= theFieldDimensions.yPosLeftGoal) /*|| (theFieldBall.positionRelative.y() < theFieldDimensions.yPosLeftGoal)*/
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(theFieldBall.positionRelative.norm() < 3000.0f)
          goto walkToBall;  
		if(theLibCheck.closerToTheBall && theFieldBall.ballWasSeen(1000))  //El que estè viendo al balon y este mas cerca.
		  goto prueba;
      }

      action
      {
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
      }
    }

    state(alignToGoal)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(theRobotPose.translation.y() <= theFieldDimensions.yPosLeftGoal)
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
          goto alignBehindBall;
		if(theLibCheck.closerToTheBall && theFieldBall.ballWasSeen(1000))  //El que estè viendo al balon y este mas cerca.
		  goto prueba;
      }

      action
      {
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
      }
    }
    state(alignBehindBall)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(theRobotPose.translation.y() <= theFieldDimensions.yPosLeftGoal)
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToGoal) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
          goto kick;
		if(theLibCheck.closerToTheBall && theFieldBall.ballWasSeen(1000))  //El que estè viendo al balon y este mas cerca.
		  goto prueba;
      }

      action
      {
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }
    // state(goBackHome){
    //   transition
    //   {
    //     if(theFieldBall.ballWasSeen())
    //       goto turnToBall;
    //     if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
    //       goto searchForBall;
    //   }
    //   action
    //   {
    //     theLookForwardSkill();
    //     thePathToTargetSkill(1.0, Defender1Pos);
    //   }
    // }



    state(kick)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(theRobotPose.translation.y() <= theFieldDimensions.yPosLeftGoal)
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;

       // if(!theFieldBall.ballWasSeen(10000))
         // goto goBackHome;  

        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto lookLeft;
        // if(!theFieldBall.ballWasSeen(10000))
        //   goto goBackHome;  

      }

      action
      {
        // LookAroundCard();
        // theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(),theFieldBall.positionRelative.y(),0.f),(HeadMotionRequest::autoCamera), 3);
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
        if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));
      }
    }

	
	state(prueba)
    {
      transition
      {

	    if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        //if(!theFieldBall.ballWasSeen(10000))
          //goto goBackHome;  
      }

      action
      {
		  
		  
        theSaySkill("LEFT");
    }
	}
	



    state(lookLeft) 
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if (theLookAtAnglesSkill.isDone())
          goto lookRight;
        
      }
      action
      {
        theLookAtAnglesSkill(pi/6, 0, 1);
        if(theLookAtAnglesSkill.isDone())
          theSaySkill("aaaaaaaaaaaaaaaa");
        // theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(),theFieldBall.positionRelative.y(),0.f),(HeadMotionRequest::targetOnGroundMode),1);


      }
    }

    state(lookRight)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(theLookAtAnglesSkill.isDone())
          goto lookLeft;
      }
      action
      {
        theLookAtAnglesSkill(-pi/3, 0, 1, HeadMotionRequest::autoCamera);
        theStandSkill();
      }
    }
  }
  

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }
};

MAKE_CARD(LeftDefenderCard);