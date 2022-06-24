/**
 * @file Striker.cpp
 *
 * 
 *
 * @author Dap
 * @author Jose P
 * @author Mia
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


CARD(StrikerCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(Say),
  CALLS(PathToTarget),
  CALLS(LookAtAngles),
  CALLS(KeyFrameArms),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(LibCheck),
  
  DEFINES_PARAMETERS(
  {,
    (float)(0.7f) walkSpeed,
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
    (Pose2f)(Pose2f(0,2000,0)) StrikerPos,
  }),
});

class StrikerCard : public StrikerCardBase
{
  bool preconditions() const override
  {
    return theRobotInfo.number == 4;
  }

  bool postconditions() const override
  {
    return theRobotInfo.number != 4;
  }

  option
  {
    theActivitySkill(BehaviorStatus::Striker);
    
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
        if(theLibCheck.LeftAttacking)
          goto receiveLeftPass;
        if(theLibCheck.RightAttacking)
          goto receiveRightPass;
        if(theLibCheck.closerToTheBall == 2)
          goto receiveCentralPass;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold || theFieldBall.positionOnField.x() > theFieldDimensions.xPosHalfWayLine)
          goto walkToBall;
      }
      action 
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
      }
    }

    state(giraCabezaDer)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout) && state_time > 1500)
          goto giraCabezaIzq;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(!theFieldBall.ballWasSeen(13000))
          goto goBackHome;
      }
      action
      {
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
        theLookAtAnglesSkill(-1,2);
      }
    }

    state(giraCabezaIzq)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout) && state_time > 1500)
          goto giraCabezaDer;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(!theFieldBall.ballWasSeen(13000))
          goto goBackHome;
      }
      action
      {
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
        theLookAtAnglesSkill(1,2);
      }
    }

    state(walkToBall)
    {
      transition
      {
        if(theLibCheck.LeftAttacking)
          goto receiveLeftPass;
        if(theLibCheck.RightAttacking)
          goto receiveRightPass;
        if(theLibCheck.closerToTheBall == 2)
          goto receiveCentralPass;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(theFieldBall.positionOnField.x() < 0)
          goto waitBall;
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
          goto alignToGoal;
      }
      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
      }
    }

    state(waitBall)
    {
      transition
      {
        if(theFieldBall.positionOnField.x() > theFieldDimensions.xPosHalfWayLine)
          goto walkToBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(theLibCheck.closerToTheBall == 2)
          goto receiveCentralPass;
      }
      action
      {
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        if(theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() + 500, 0.f));
      }
    }

    state(goBackHome)
    {
      transition
      {
        if(theLibCheck.LeftAttacking)
          goto receiveLeftPass;
        if(theLibCheck.RightAttacking)
          goto receiveRightPass;
        if(theLibCheck.closerToTheBall == 2)
          goto receiveCentralPass;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
      }
      action
      {
        theLookForwardSkill();
        thePathToTargetSkill(1.0, StrikerPos);
      }
    }

    state(receiveLeftPass)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(theFieldBall.positionRelative.norm() < 200.f)
          goto alignToGoal;
        if(!theLibCheck.LeftAttacking)
          goto walkToBall;
      }
      action
      {
        theLookForwardSkill();
        thePathToTargetSkill(walkSpeed, (0.f, Pose2f(3000, -1500)));
        if(theRobotPose.translation == Pose2f(3000, -1500))
          theSaySkill("Left pass");
      }
    }

    state(receiveRightPass)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(theFieldBall.positionRelative.norm() < 200.f)
          goto alignToGoal;
        if(!theLibCheck.RightAttacking)
          goto walkToBall;
      }
      action
      {
        theLookForwardSkill();
        thePathToTargetSkill(walkSpeed, (0.f, Pose2f(3000, 1500)));
        if(theRobotPose.translation == Pose2f(3000, 1500))
          theSaySkill("right pass");
      }
    }

    state(receiveCentralPass)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer; 
        if(theLibCheck.positionToPass && theFieldBall.positionRelative.norm() < 500.f)
          goto alignToGoal;
      }
      action
      {
        thePathToTargetSkill(walkSpeed,Pose2f(pi,500.f,1000.f));
        if(theRobotPose.translation == Pose2f(500, 1000))
          theSaySkill("central pass");
      }
    }

    state(alignToGoal)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
          goto waitBall;
      }
      action
      {
        theSaySkill("align goal");
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
        if(theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() + 500, 0.f));
      }
    }

    state(alignBehindBall)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(std::abs(angleToGoal) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
          goto kickAtGoal;
      }
      action
      {
        theSaySkill("behind");
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX-20.f, theFieldBall.positionRelative.y() - ballOffsetY));
        if(theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() + 500, 0.f));
      }
    }

    state(kickAtGoal)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
      }
      action
      {
        theSaySkill("kick");
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theKickSkill((KickRequest::kickForward), true,0.2f, false);
      }
    }
  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  } 

  int randomNum() 
  {
    int random = 0;
    random = rand()%(2 + 0);
    if(theRobotPose.translation.y() > (theFieldDimensions.yPosLeftFieldBorder - 1500.f))
      random = 0;
    if(theRobotPose.translation.y() < (theFieldDimensions.yPosRightFieldBorder + 1500.f))
      random = 1;

    return random;
  } 

};

MAKE_CARD(StrikerCard);
