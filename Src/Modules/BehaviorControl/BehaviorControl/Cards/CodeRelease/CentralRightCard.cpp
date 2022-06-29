/**
 * @file DefenderCard.cpp
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
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Modeling/Obstacle.h"
#include "Representations/Communication/TeamData.h"


CARD(CentralRightCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(LookAtAngles),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(KeyFrameArms),
  CALLS(PathToTarget),
  CALLS(SpecialAction),
  CALLS(Say),
  CALLS(PassTarget),
  CALLS(Kick),
  REQUIRES(TeamData),
  REQUIRES(ObstacleModel),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(RobotInfo),
  REQUIRES(LibCheck),
  DEFINES_PARAMETERS(
  {,
    (float)(1.0f) walkSpeed,
    (int)(500) initialWaitTime,
    (int)(4000) ballNotSeenTimeout,
    (Pose2f)(Pose2f(0,-3000,-1000)) Defender1Pos,
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
    (int)(0) a,
    (std::vector<Teammate>) OpponentRobots,
  }),
});

class CentralRightCard : public CentralRightCardBase
{
  bool preconditions() const override
  {
    return theRobotInfo.number == (theLibCheck.rightEnter);
  }

  bool postconditions() const override
  {
    return theRobotInfo.number != (theLibCheck.rightEnter);
  }

  option
  {
    theActivitySkill(BehaviorStatus::CentralDefender);

    bool hayObstaculoCerca = false;
      if(!theObstacleModel.obstacles.empty()){     //Tenemos obstàculos, entonces, actuamos.   
      for(const auto& obstacle : theObstacleModel.obstacles){
        //See if the obstacle is first than the target   
      if (obstacle.center.norm()<400.f)  
          hayObstaculoCerca=true;
      }
      if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));
    }
    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto goBackHome;
      }

      action
      {
        theSaySkill("Central Defend Right Change card");
        theLookAtAnglesSkill(theFieldBall.positionRelative.angle(),2);
        theStandSkill();
      }
    }

    state(turnToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer;  
        if(((std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold) || (theFieldBall.positionRelative.x() < theFieldDimensions.xPosHalfWayLine)) && !hayObstaculoCerca)
          goto DefendBall;  
        if(hayObstaculoCerca)
          goto ObsAvoid;
      }  
      action
      {
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
        if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));
      }
    }  

    state(walkToBall)
    {
      transition
      {
        if((theRobotPose.translation.x() >= theFieldDimensions.xPosHalfWayLine) || (theFieldBall.positionRelative.x()*-1 >= 0 ))
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        // if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold) && !hayObstaculoCerca && !hayObstaculoLejos)
        //   goto alignToPass;
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold) && !theLibCheck.LeftDefending && !theLibCheck.RightDefending)
          goto alignToGoal;
        if(hayObstaculoCerca)
          goto ObsAvoid;  
      }

      action
      {
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
        if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));        
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
        if(theRobotPose.translation.x() < 0)
          goto DefendBall;  
      }    
      action
      {
        
        if(theLibCheck.LeftAttacking)
          theSaySkill("left attack");
        if(theLibCheck.LeftDefending)
          theSaySkill("SIUUUUUU");
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));
      }
    }

    state(DefendBall)
    {
      transition
      {
        if((theRobotPose.translation.x() >= theFieldDimensions.xPosHalfWayLine) || (theFieldBall.positionRelative.x()*-1 >= 0))
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(theFieldBall.positionRelative.norm() < 3000.0f && !hayObstaculoCerca && !theLibCheck.LeftDefending && !theLibCheck.RightDefending)
          goto walkToBall;  
        if(hayObstaculoCerca)
          goto ObsAvoid;
      }

      action
      {
        
        if(theLibCheck.LeftAttacking)
          theSaySkill("left attack");
        if(theLibCheck.LeftDefending)
          theSaySkill("SIUUUUUU");
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));
      }
    }

    state(alignToGoal)
    {
      const Angle angleToGoal = calcAngleToGoal();
      int random = randomNum();

      transition
      {
        if((theRobotPose.translation.x() >= theFieldDimensions.xPosHalfWayLine) || (theFieldBall.positionRelative.x()*-1 >= 0))
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold && random == 0)
          goto alignBehindBallRight; 
        if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold && random == 1)
          goto alignBehindBallLeft; 
      }

      action
      {
        if(theLibCheck.LeftAttacking)
          theSaySkill("left attack");
        if(theLibCheck.LeftDefending)
          theSaySkill("SIUUUUUU");
        theSaySkill("Align Goal");
        theLookAtAnglesSkill(theFieldBall.positionRelative.angle(),2);
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
        if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));      
      }
    }
    state(alignBehindBallRight)
    {
      const Angle angleToGoal = calcAngleToGoal();
      //const Angle angleToTeammate = calcAngleToTeammate();
      bool hayObstaculos = hayObstaculo();

      transition
      {
        if((theRobotPose.translation.x() >= theFieldDimensions.xPosHalfWayLine) || (theFieldBall.positionRelative.x()*-1 >= 0))
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(!theFieldBall.ballWasSeen(300))
          goto kickRight;
        if(!hayObstaculos && theLibCheck.positionToPass)
          goto alignToPass;
        if(!hayObstaculos && !theLibCheck.positionToPass)  
          goto alignToClearance;
      }

      action
      {
        theLookForwardSkill();
        theSaySkill("zero");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX + 45.f, theFieldBall.positionRelative.y() - ballOffsetY + 200.f));
        if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));
      }
    }

  state(alignBehindBallLeft)
    {
      const Angle angleToGoal = calcAngleToGoal();
      //const Angle angleToTeammate = calcAngleToTeammate();
      bool hayObstaculos = hayObstaculo();

      transition
      {
        if((theRobotPose.translation.x() >= theFieldDimensions.xPosHalfWayLine) || (theFieldBall.positionRelative.x()*-1 >= 0))
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer;  
        if(!theFieldBall.ballWasSeen(300))
          goto kickLeft;  
        if(!hayObstaculos && theLibCheck.positionToPass)
          goto alignToPass;
        if(!hayObstaculos && !theLibCheck.positionToPass)  
          goto alignToClearance;
      }

      action
      {
        theLookForwardSkill();
        theSaySkill("one");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX + 45.f, theFieldBall.positionRelative.y() + ballOffsetY - 200.f));
        if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));
      }
    }

    state(alignBehindBallToPass)
    {
      const Angle angleToTeammate = calcAngleToTeammate();
      bool hayObstaculos = hayObstaculo();

      transition
      {
        if(std::abs(angleToTeammate) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()) && !hayObstaculos)
          goto pass; 
        if(hayObstaculos)
          goto alignToGoal;
      }
      action
      {
        theLookForwardSkill();
        theSaySkill("Behind Pass");
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToTeammate, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));
      }
    }

    state(goBackHome){
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(hayObstaculoCerca)
          goto ObsAvoid;
      }
      action
      {
        thePathToTargetSkill(1.0, Defender1Pos);
      }
    }

    state(kickRight)
    {

      transition
      {
        if((theRobotPose.translation.x() >= theFieldDimensions.xPosHalfWayLine) || (theFieldBall.positionRelative.x()*-1 >= 0))
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theSaySkill("kick zero");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() - 2000));
        if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));
      }
    }

      state(kickLeft)
    {

      transition
      {
        if((theRobotPose.translation.x() >= theFieldDimensions.xPosHalfWayLine) || (theFieldBall.positionRelative.x()*-1 >= 0))
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theSaySkill("kick one");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() + 2000));
        if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));
      }
    }

    state(pass)
    {
      bool hayObstaculos = hayObstaculo();

      transition
      {
        if((theRobotPose.translation.x() >= theFieldDimensions.xPosHalfWayLine) || (theFieldBall.positionRelative.x()*-1 >= 0))
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
        if(hayObstaculos)
          goto alignToGoal;
      }
      action
      {
        theSaySkill("pass");
        // if(theLibCheck.positionToPass)
        //   theSaySkill("Yes");
        // else
        //   theSaySkill("No");
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theKickSkill((KickRequest::kickForward), true,0.2f, false);
        if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));
      }
    }
    
    state(alignToPass)
    {
      const Angle angleToTeammate = calcAngleToTeammate();
      const Angle angleToClearance = calcAngleClearance();
      bool hayObstaculos = hayObstaculo();

      transition
      {
        if((theRobotPose.translation.x() >= theFieldDimensions.xPosHalfWayLine) || (theFieldBall.positionRelative.x()*-1 >= 0))
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(std::abs(angleToTeammate) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
          goto alignBehindBallToPass;
        if(hayObstaculos)
          goto alignToGoal;
      }

      action
      {
        theSaySkill("Align Pass");
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        if(theLibCheck.positionToPass)
          theSaySkill("to striker");
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToTeammate, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
        if(!theLibCheck.positionToPass)
          theSaySkill("clearance");
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToClearance, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
        if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));
      }
    }

    state(alignToClearance)
    {
      const Angle angleToClearance = calcAngleClearance();
      bool hayObstaculos = hayObstaculo();

      transition
      {
        if((theRobotPose.translation.x() >= theFieldDimensions.xPosHalfWayLine) || (theFieldBall.positionRelative.x()*-1 >= 0))
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(std::abs(angleToClearance) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
          goto alignBehindBallToClearance;
        if(hayObstaculos)
          goto alignToGoal;
      }

      action
      {
        theSaySkill("Align Clearance");
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToClearance, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
        if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));
      }
    }
    state(alignBehindBallToClearance)
    {
      const Angle angleToClearance = calcAngleClearance();
      bool hayObstaculos = hayObstaculo();

      transition
      {
        if(std::abs(angleToClearance) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()) && !hayObstaculos)
          goto pass; 
        if(hayObstaculos)
          goto alignToGoal;
      }
      action
      {
        theLookForwardSkill();
        theSaySkill("Behind Clearance");
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToClearance, theFieldBall.positionRelative.x() - ballOffsetX  - 17, theFieldBall.positionRelative.y() - ballOffsetY));
        if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));
      }
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(!theFieldBall.ballWasSeen(10000))
          goto goBackHome; 
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
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
        if(theRobotPose.translation.x() > theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theRobotPose.inversePose.translation.x() - 500, 0.f));
      }
    }

    state(GiraCabezaIzq)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout) && state_time > 1500)
          goto GiraCabezaDer;
        if(hayObstaculoCerca)
          goto ObsAvoid;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(a > 2)
          goto searchForBall;
      }
      action
      {
        
          theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
          theLookAtAnglesSkill(1,2);
          //a+=1;   
      }
    } 

    state(GiraCabezaDer)
    {
      
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout) && state_time > 1500)
          goto GiraCabezaIzq;
        if(hayObstaculoCerca)
          goto ObsAvoid;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;   
      }

      action
      {
        
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
        theLookAtAnglesSkill(-1,2);
      }
    } 
  }
  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }

  Angle calcAngleToTeammate() const
  {
    return (theRobotPose.inversePose * Vector2f(500.f,1000.f)).angle();
  }

  Angle calcAngleClearance() const
  {
    return (theRobotPose.inversePose * Vector2f(0.f,0.f)).angle();
  }

  bool hayObstaculo() 
  {
    bool x = false;
    if(!theObstacleModel.obstacles.empty()){     //Tenemos obstàculos, entonces, actuamos.   
      for(const auto& obstacle : theObstacleModel.obstacles){
        //See if the obstacle is first than the target   
      if(obstacle.center.norm() < 700.f && (obstacle.center.y() < 100 && obstacle.center.y() > -100))
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

MAKE_CARD(CentralRightCard);