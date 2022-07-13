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
  CALLS(SpecialAction),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(LibCheck),
  
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
      if (obstacle.center.norm()<200.f)  
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
        if(theLibCheck.closerToTheBall == 3)
          goto receiveLeftPass;
        if(theLibCheck.closerToTheBall == 5)
          goto receiveRightPass;
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine - 300 && !theLibCheck.LeftAttacking && !theLibCheck.RightAttacking && theLibCheck.closerToTheBall != 2 && theLibCheck.closerToTheBall != 4)
          goto goToCenter;
        if(theLibCheck.closerToTheBall == 2)
          goto receiveCentralPass;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(theFieldBall.positionRelative.norm() < 4000.0f && theFieldBall.positionOnField.x() > theFieldDimensions.xPosHalfWayLine - 1000)
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
        if(theLibCheck.closerToTheBall == 3 && theLibCheck.LeftAttacking)
          goto receiveLeftPass;
        if(theLibCheck.closerToTheBall == 5 && theLibCheck.RightAttacking)
          goto receiveRightPass;
        if(theLibCheck.closerToTheBall == 2)
          goto receiveCentralPass;
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
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout) && state_time > 1500)
          goto giraCabezaDer;
        if(theLibCheck.closerToTheBall == 3 && theLibCheck.LeftAttacking)
          goto receiveLeftPass;
        if(theLibCheck.closerToTheBall == 5 && theLibCheck.RightAttacking)
          goto receiveRightPass;
        if(theLibCheck.closerToTheBall == 2)
          goto receiveCentralPass;
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
        if(theLibCheck.closerToTheBall == 3 && theLibCheck.LeftAttacking)
          goto receiveLeftPass;
        if(theLibCheck.closerToTheBall == 5 && theLibCheck.RightAttacking)
          goto receiveRightPass;
        if(theLibCheck.closerToTheBall == 2)
          goto receiveCentralPass;
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine - 300 && !theLibCheck.LeftAttacking && !theLibCheck.RightAttacking && theLibCheck.closerToTheBall != 2 )
          goto goToCenter;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
          goto alignToGoal;
      }
      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
      }
    }

    state(avanceConBalon)
    {
      const Angle angleToGoal = calcAngleToGoal();
      bool hayObstaculos = hayObstaculo();

      transition
      {
        if(theRobotPose.translation.x() > 2500.f || hayObstaculos)
          goto alignToGoal;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(theLibCheck.closerToTheBall == 3)
          goto receiveLeftPass;
        if(theLibCheck.closerToTheBall == 5)
          goto receiveRightPass;
        if(theLibCheck.closerToTheBall == 2)
          goto receiveCentralPass;
      }
      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed + 0.2f, walkSpeed + 0.2f, walkSpeed + 0.2f), Pose2f(angleToGoal, theFieldBall.positionRelative.x() + 40 - ballOffsetX, theFieldBall.positionRelative.y() + ballOffsetY/2));
      }
    }

    state(goToCenter)
    {
      transition
      {
        if(hayObstaculoCerca)
          goto obsAvoid;
        if(theLibCheck.closerToTheBall == 3)
          goto receiveLeftPass;
        if(theLibCheck.closerToTheBall == 5)
          goto receiveRightPass;
        if(theLibCheck.closerToTheBall == 2)
          goto receiveCentralPass;
        if(theFieldBall.positionOnField.x() > theFieldDimensions.xPosHalfWayLine && !theLibCheck.LeftAttacking && !theLibCheck.RightAttacking)
          goto walkToBall;
      }
      action
      {
        theLookForwardSkill();
        thePathToTargetSkill(walkSpeed + 0.3f, Pose2f(theFieldBall.positionRelative.angle(), 2000, 0));
      }
    }

    state(receiveLeftPass)
    {
      const Angle angleToPass = calcAngleToPass();

      transition
      {
        if(hayObstaculoCerca)
          goto obsAvoid;
        if(!theLibCheck.LeftAttacking || (theLibCheck.closerToTheBall != 2 && theLibCheck.closerToTheBall != 3 && theLibCheck.closerToTheBall != 5))
          goto walkToBall;
        if(theLibCheck.closerToTheBall == 2 && !theLibCheck.LeftAttacking)
          goto receiveCentralPass;
        if(theRobotPose.translation.x() >= 2500 && theRobotPose.translation.x() < 3500 && theRobotPose.translation.y() >= -2000 && theRobotPose.translation.y() < -1000)
          goto waitAtSidePass;
      }
      action
      {
        theLookForwardSkill();
        thePathToTargetSkill(walkSpeed + 0.3f, Pose2f(angleToPass, 3000, -1500));
      }
    }

    state(receiveRightPass)
    {
      const Angle angleToPass = calcAngleToPass();

      transition
      {
        if(hayObstaculoCerca)
          goto obsAvoid;
        if(!theLibCheck.RightAttacking || (theLibCheck.closerToTheBall != 2 && theLibCheck.closerToTheBall != 3 && theLibCheck.closerToTheBall != 5))
          goto walkToBall;
        if(theLibCheck.closerToTheBall == 2 && !theLibCheck.RightAttacking)
          goto receiveCentralPass;
        if(theRobotPose.translation.x() >= 2500 && theRobotPose.translation.x() < 3500 && theRobotPose.translation.y() >= 1000 && theRobotPose.translation.y() < 2000)
          goto waitAtSidePass;
      }
      action
      {
        theLookForwardSkill();
        thePathToTargetSkill(walkSpeed + 0.3f, Pose2f(angleToPass, 3000, 1500));
      }
    }

    state(receiveCentralPass)
    {
      const Angle angleToPass = calcAngleToCentralPass();

      transition
      {
        if(hayObstaculoCerca)
          goto obsAvoid;
        if(theLibCheck.closerToTheBall == 3)
          goto receiveLeftPass;
        if(theLibCheck.closerToTheBall == 5)
          goto receiveRightPass;
        if(theRobotPose.translation.x() >= 1500 && theRobotPose.translation.x() < 2000 && theRobotPose.translation.y() >= 500 && theRobotPose.translation.y() < 1500)
          goto waitAtCentralPass;
        if(theFieldBall.positionRelative.norm() > 500.f && theFieldBall.positionOnField.x() > theFieldDimensions.xPosHalfWayLine)
          goto walkToBall;
      }
      action
      {
        theLookForwardSkill();
        thePathToTargetSkill(walkSpeed + 0.3f,Pose2f(angleToPass, 2000.f, 1000.f));
      }
    }

    state(waitAtSidePass)
    {
      const Angle angleToPass = calcAngleToPass();

      transition
      {
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine && !theLibCheck.LeftAttacking && !theLibCheck.RightAttacking && theLibCheck.closerToTheBall != 2 )
          goto goToCenter;
        if(theFieldBall.positionRelative.norm() < 1500.f && theFieldBall.positionOnField.x() > theFieldDimensions.xPosHalfWayLine && ((!theLibCheck.LeftAttacking && !theLibCheck.RightAttacking)))
          goto walkToBall;
        if(!theFieldBall.ballWasSeen(8000))
          goto giraCabezaDer;
      }
      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToPass, 0.f, 0.f));
      }
    }

    state(waitAtCentralPass)
    {
      const Angle angleToPass = calcAngleToCentralPass();

      transition
      {
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine && !theLibCheck.LeftAttacking && !theLibCheck.RightAttacking && theLibCheck.closerToTheBall != 2 )
          goto goToCenter;
        if(theFieldBall.positionRelative.norm() < 1500.f && theFieldBall.positionOnField.x() > theFieldDimensions.xPosHalfWayLine && ((!theLibCheck.LeftAttacking && !theLibCheck.RightAttacking)))
          goto walkToBall;
        if(!theFieldBall.ballWasSeen(8000))
          goto giraCabezaDer;
      }
      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToPass, 0.f, 0.f));
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
        if(theRobotPose.translation.x() <= 2500.f && !hayObstaculos)
          goto avanceConBalon;
        if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold && hayObstaculos && random == 0)
          goto alignRight;
        if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold && hayObstaculos && random == 1)
          goto alignLeft;
        if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold && !hayObstaculos)
          goto alignBehindBall;
      }
      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed + 0.3f, walkSpeed + 0.3f, walkSpeed + 0.3f), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
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
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed + 0.4f, walkSpeed + 0.4f, walkSpeed + 0.4f), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX-20.f, theFieldBall.positionRelative.y() - ballOffsetY));
        if(theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine - 300)
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
          goto goToCenter;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer; 
        if(!hayObstaculos)
          goto alignToGoal;
        if(!theFieldBall.ballWasSeen(300))
          goto kickRight;
      }
      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed + 0.4f, walkSpeed + 0.4f, walkSpeed + 0.4f), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX + 45.f, theFieldBall.positionRelative.y() - ballOffsetY + 200.f));
        if(theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine - 300)
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
          goto goToCenter;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(!hayObstaculos)
          goto alignToGoal;
        if(!theFieldBall.ballWasSeen(300))
          goto kickLeft;
      }
      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed + 0.4f, walkSpeed + 0.4f, walkSpeed + 0.4f), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX + 45.f, theFieldBall.positionRelative.y() - ballOffsetY - 200.f));
        if(theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine - 300)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() + 500, 0.f));
      }
    }

    state(kickAtGoal)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
      }
      action
      {
        theLookForwardSkill();
        if(theRobotPose.translation.x() < theFieldDimensions.xPosOpponentPenaltyMark + 500)
          theKickSkill((KickRequest::kickForwardFastLong), true,0.2f, false);
        if(theRobotPose.translation.x() >= theFieldDimensions.xPosOpponentPenaltyMark + 500)
          theInWalkKickSkill(WalkKickVariant(WalkKicks::forwardOLD, Legs::left), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }

    state(kickRight)
    {
      bool hayObstaculos = hayObstaculo();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
        if(!hayObstaculos)
          goto alignToGoal;
      }
      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed + 0.4f, walkSpeed + 0.4f, walkSpeed + 0.4f), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() - 2000));
      }
    }

    state(kickLeft)
    {
      bool hayObstaculos = hayObstaculo();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giraCabezaDer;
          if(!hayObstaculos)
          goto alignToGoal;
      }
      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed + 0.4f, walkSpeed + 0.4f, walkSpeed + 0.4f), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() + 2000));
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
          goto goToCenter;
      }
      action
      {
        for(const auto& obstacle : theObstacleModel.obstacles) {
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, obstacle.center.norm()+100)); 
        }

        if(theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine - 300)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() + 500, 0.f));
      }
    }
  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }

  Angle calcAngleToPass() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, theFieldDimensions.yPosCenterGoal)).angle();
  }

  Angle calcAngleToCentralPass() const
  {
    return (theRobotPose.inversePose * Vector2f(100.f, 100.f)).angle();
  }

  bool hayObstaculo()
  {
    bool x = false;
    if(!theObstacleModel.obstacles.empty()){     //Tenemos obstàculos, entonces, actuamos.   
      for(const auto& obstacle : theObstacleModel.obstacles){
        //See if the obstacle is first than the target   
      if(obstacle.center.norm() < 500.f && (obstacle.center.y() < 150 && obstacle.center.y() > -150))
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
