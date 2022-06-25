/**
 * @file Striker.cpp
 *
 * 
 * @author Mia
 * @author Dap
 * @author Jose P
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
    (float)(0.9f) walkSpeed,
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
        if(hayObstaculoCerca)
          goto obsAvoid;
        if(theLibCheck.LeftAttacking)
          goto receiveLeftPass;
        if(theLibCheck.RightAttacking)
          goto receiveRightPass;
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
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
        if(hayObstaculoCerca)
          goto obsAvoid;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout) && state_time > 1500)
          goto giraCabezaIzq;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
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
        if(hayObstaculoCerca)
          goto obsAvoid;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout) && state_time > 1500)
          goto giraCabezaDer;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
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
        if(hayObstaculoCerca)
          goto obsAvoid;
        if(theLibCheck.LeftAttacking)
          goto receiveLeftPass;
        if(theLibCheck.RightAttacking)
          goto receiveRightPass;
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
          goto receiveCentralPass;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
          goto alignToGoal;
      }
      action
      {
        theSaySkill("walk");
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
      }
    }

    state(receiveLeftPass)
    {
      transition
      {
        if(hayObstaculoCerca)
          goto obsAvoid;
        if(theFieldBall.positionRelative.norm() < 500.f)
          goto alignToGoal;
        if(!theLibCheck.LeftAttacking)
          goto walkToBall;
      }
      action
      {
        theSaySkill("left");
        theLookForwardSkill();
        thePathToTargetSkill(walkSpeed, Pose2f(pi/2, 3000, -1500));
      }
    }

    state(receiveRightPass)
    {
      transition
      {
        if(hayObstaculoCerca)
          goto obsAvoid;
        if(theFieldBall.positionRelative.norm() < 500.f)
          goto alignToGoal;
        if(!theLibCheck.RightAttacking)
          goto walkToBall;
      }
      action
      {
        theSaySkill("right");
        theLookForwardSkill();
        thePathToTargetSkill(walkSpeed, Pose2f(3*pi/2, 3000, 1500));
      }
    }

    state(receiveCentralPass)
    {
      transition
      {
        if(hayObstaculoCerca)
          goto obsAvoid;
        if(theLibCheck.LeftAttacking && theLibCheck.closerToTheBall != 2)
          goto receiveLeftPass;
        if(theLibCheck.RightAttacking && theLibCheck.closerToTheBall != 2)
          goto receiveRightPass;
        if(theLibCheck.positionToPass && theFieldBall.positionRelative.norm() < 500.f)
          goto alignToGoal;
        if(theFieldBall.positionRelative.norm() > 500.f && theFieldBall.positionOnField.x() > theFieldDimensions.xPosHalfWayLine)
          goto walkToBall;
      }
      action
      {
        theSaySkill("central");
        thePathToTargetSkill(walkSpeed,Pose2f(pi,2000.f,1000.f));
      }
    }

    state(alignToGoal)
    {
      const Angle angleToGoal = calcAngleToGoal();
      bool hayObstaculos = hayObstaculo();
      int random = randomNum();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
          goto receiveCentralPass;
        if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold && !hayObstaculos)
          goto alignBehindBall;
        if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold && hayObstaculos && random == 0)
          goto alignRight;
        if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold && hayObstaculos && random == 0)
          goto alignLeft;
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
        if(theRobotPose.translation.x() < 1500.f)
          goto closerToGoal;
        if(std::abs(angleToGoal) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()) && theRobotPose.translation.x() > 1500.f)
          goto kickAtGoal;
      }
      action
      {
        theSaySkill("behind");
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        if(theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() + 500, 0.f));
      }
    }

    state(alignRight)
    {
      bool hayObstaculos = hayObstaculo();
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
          goto receiveCentralPass;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer; 
        if(!hayObstaculos)
          goto alignToGoal;
        if(!theFieldBall.ballWasSeen(300))
          goto kickRight;
      }
      action
      {
        theSaySkill("align right");
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX + 45.f, theFieldBall.positionRelative.y() - ballOffsetY + 200.f));
        if(theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() + 500, 0.f));
      }
    }

    state(alignLeft)
    {
      bool hayObstaculos = hayObstaculo();
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
          goto receiveCentralPass;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(!hayObstaculos)
          goto alignToGoal;
        if(!theFieldBall.ballWasSeen(300))
          goto kickLeft;
      }
      action
      {
        theSaySkill("align left");
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX + 45.f, theFieldBall.positionRelative.y() - ballOffsetY - 200.f));
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
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
          goto receiveCentralPass;
        if(theRobotPose.translation.x() < 1500.f)
          goto closerToGoal;
      }
      action
      {
        theSaySkill("kick");
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theKickSkill((KickRequest::kickForward), true,0.2f, false);
      }
    }

    state(kickRight)
    {
      bool hayObstaculos = hayObstaculo();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
          goto receiveCentralPass;
        if(!hayObstaculos)
          goto alignToGoal;
      }
      action
      {
        theLookForwardSkill();
        theSaySkill("kick right");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() - 2000));
      }
    }

    state(kickLeft)
    {
      bool hayObstaculos = hayObstaculo();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
          goto receiveCentralPass;
          if(!hayObstaculos)
          goto alignToGoal;
      }
      action
      {
        theLookForwardSkill();
        theSaySkill("kick left");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() + 2000));
      }
    }

    state(closerToGoal)
    {
      bool hayObstaculos = hayObstaculo();
      const Angle angleToGo = calcAngleToGo();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
          goto receiveCentralPass;
        if(theRobotPose.translation.x() > 1500)
          goto alignBehindBall;
        if(hayObstaculos)
          goto alignToGoal;
      }
      action
      {
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed + 0.2f, walkSpeed + 0.2f, walkSpeed + 0.2f), Pose2f(angleToGo, theFieldBall.positionRelative.x() + 40 - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY/2));
      }
    }

    state(obsAvoid)
    {  
      transition 
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
          goto receiveCentralPass;
      }
      action
      {
        for(const auto& obstacle : theObstacleModel.obstacles) {
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, obstacle.center.norm()+100)); 
        }

        if(theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() + 500, 0.f));
      }
    }
  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }

  Angle calcAngleToGo() const
  {
    return (theRobotPose.inversePose * Vector2f(5000.f,theFieldDimensions.yPosLeftGoal)).angle();
  }

  bool hayObstaculo() const
  {
    bool x = false;
    if(!theObstacleModel.obstacles.empty()){     //Tenemos obstàculos, entonces, actuamos.   
      for(const auto& obstacle : theObstacleModel.obstacles){
        //See if the obstacle is first than the target   
      if(obstacle.center.norm() < 850.f && (obstacle.center.y() < 400 && obstacle.center.y() > -400))
        x = true;
      }
    }
    return x;
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
