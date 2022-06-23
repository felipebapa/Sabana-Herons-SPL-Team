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
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Communication/RobotInfo.h"
#include "Tools/Modeling/Obstacle.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Communication/TeamData.h"


CARD(LeftDefenderCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(Kick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(KeyFrameArms),
  CALLS(PathToTarget),
  CALLS(Say),
  CALLS(LookAtAngles),
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
    (int)(0) a,
    (std::vector<Teammate>) OpponentRobots,
  }),
});

class LeftDefenderCard : public LeftDefenderCardBase
{
  bool preconditions() const override
  {
    return theRobotInfo.number == (theLibCheck.leftLeave);
  }

  bool postconditions() const override
  {
    return theRobotInfo.number != (theLibCheck.leftLeave);
  }

  option
  {
    theActivitySkill(BehaviorStatus::LeftDefender);

    bool hayObstaculoCerca = false;
      if(!theObstacleModel.obstacles.empty()){     //Tenemos obstàculos, entonces, actuamos.   
      for(const auto& obstacle : theObstacleModel.obstacles){
        //See if the obstacle is first than the target   
      if (obstacle.center.norm()<400.f)  
          hayObstaculoCerca=true;
      }
    }

    bool OpponentCloserOwnGoal = false;

      for(auto const& oponente : theTeamData.teammates){

        if(oponente.mateType==Teammate::otherTeamRobot){

            OpponentRobots.push_back(oponente);  //Lista sin ordenar de oponentes.
            //teammate.theRobotPose.translation.x()>=theFieldDimensions.xPosHalfWayLine

            if(oponente.theRobotPose.translation.x()<=theFieldDimensions.xPosOwnGoal+200){

              
                OpponentCloserOwnGoal=true;

            }
        }

      }

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto turnToBall;
        if(hayObstaculoCerca)
          goto ObsAvoid;
      }

      action
      {
        theSaySkill("ONE");
        //theLookForwardSkill();
        theLookAtAnglesSkill(theFieldBall.positionRelative.angle(),2);
        theStandSkill();

        if(!OpponentRobots.empty()){  

            theSaySkill("Opponent");

        } 
      }
    }

    state(turnToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold && !hayObstaculoCerca)
          goto DefendBall;
        if(hayObstaculoCerca)
          goto ObsAvoid;
      }  
      action
      {
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
        if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal && theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));
      }
    }  

    state(walkToBall)
    {
      transition
      {
        // if(theLibCheck.iFell == 2 && (theFieldBall.positionOnField.y() < theFieldDimensions.yPosLeftGoal && theFieldBall.positionOnField.y() > theFieldDimensions.yPosRightGoal))
        //   goto centralFallen;
        if(theRobotPose.translation.y() <= theFieldDimensions.yPosLeftGoal)
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold) && !hayObstaculoCerca)
          goto alignToGoal;
        if(hayObstaculoCerca)
          goto ObsAvoid;
          
      }

      action
      {
        theSaySkill("walk");
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
        if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal && theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));
      }
    }

    state(waitBall)
    {
      transition
      {
        // if(theLibCheck.iFell == 2 && (theFieldBall.positionOnField.y() < theFieldDimensions.yPosLeftGoal && theFieldBall.positionOnField.y() > theFieldDimensions.yPosRightGoal))
        //   goto centralFallen;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(!theFieldBall.ballWasSeen(10000))
          goto goBackHome;   
      }

      action
      {
          theLookForwardSkill();
          theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
          if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal && theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));
      }
    }

    state(DefendBall)
    {
      transition
      {
        // if(theLibCheck.iFell == 2 && (theFieldBall.positionOnField.y() < theFieldDimensions.yPosLeftGoal && theFieldBall.positionOnField.y() > theFieldDimensions.yPosRightGoal))
        //   goto centralFallen;
        if(theRobotPose.translation.y() <= theFieldDimensions.yPosLeftGoal) /*|| (theFieldBall.positionRelative.y() < theFieldDimensions.yPosLeftGoal)*/
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(theFieldBall.positionRelative.norm() < 3000.0f && !hayObstaculoCerca)
          goto walkToBall;  
        if(hayObstaculoCerca)
          goto ObsAvoid;
      }

      action
      {
        theSaySkill("defend");
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal && theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));
      }
    }

    state(alignToGoal)
    {
      const Angle angleToGoal = calcAngleToGoal();
      int random = randomNum();
      bool hayObstaculos = hayObstaculo();

      transition
      {
        if(theRobotPose.translation.y() <= theFieldDimensions.yPosLeftGoal)
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold && hayObstaculos && random == 0)
          goto alignBehindBallRight;
        if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold && hayObstaculos && random == 1) 
          goto alignBehindBallLeft;
        if(!hayObstaculos)
          goto alignToPass;
      }

      action
      {
        theSaySkill("Align Goal");
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
        if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal && theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));      
      }
    }
    state(alignBehindBallRight)
    {
      const Angle angleToGoal = calcAngleToGoal();
      const Angle angleToTeammate = calcAngleToTeammate();
      const bool hayObstaculos = hayObstaculo();

      transition
      {
        // if(theLibCheck.iFell == 2 && (theFieldBall.positionOnField.y() < theFieldDimensions.yPosLeftGoal && theFieldBall.positionOnField.y() > theFieldDimensions.yPosRightGoal))
        //   goto centralFallen;
        if(theRobotPose.translation.y() <= theFieldDimensions.yPosLeftGoal)
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(!theFieldBall.ballWasSeen(500))
          goto kickRight;  
      }

      action
      {
        theLookForwardSkill();
        theSaySkill("zero");
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX + 45.f, theFieldBall.positionRelative.y() - ballOffsetY + 200.f));
        if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal && theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));      
      }
    }

    state(alignBehindBallLeft)
    {
      const Angle angleToGoal = calcAngleToGoal();
      const Angle angleToTeammate = calcAngleToTeammate();
      const bool hayObstaculos = hayObstaculo();

      transition
      {
        // if(theLibCheck.iFell == 2 && (theFieldBall.positionOnField.y() < theFieldDimensions.yPosLeftGoal && theFieldBall.positionOnField.y() > theFieldDimensions.yPosRightGoal))
        //   goto centralFallen;
        if(theRobotPose.translation.y() <= theFieldDimensions.yPosLeftGoal)
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(!theFieldBall.ballWasSeen(500))
          goto kickLeft;  
      }

      action
      {
        theLookForwardSkill();
        theSaySkill("one");
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX + 45.f, theFieldBall.positionRelative.y() + ballOffsetY - 200.f));
        if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal && theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));      
      }
    }

    state(alignBehindBallToPass)
    {
      const Angle angleToTeammate = calcAngleToTeammate();
      const bool hayObstaculos = hayObstaculo();
      const Angle angleToGo = calcAngleToGo();

      transition
      {
        // if(theLibCheck.iFell == 2 && (theFieldBall.positionOnField.y() < theFieldDimensions.yPosLeftGoal && theFieldBall.positionOnField.y() > theFieldDimensions.yPosRightGoal))
        //   goto centralFallen;
        if(hayObstaculos)
          goto alignToGoal;
        // if(std::abs(angleToTeammate) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()) && !hayObstaculos)
        //   goto avanceConBalon;
        
      }
      action
      {
        theLookForwardSkill();
        theSaySkill("go go go");
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed + 0.2f, walkSpeed + 0.2f, walkSpeed + 0.2f), Pose2f(angleToGo, theFieldBall.positionRelative.x() + 40 - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal && theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));
      }
    }

    state(goBackHome){
      transition
      {
        // if(theLibCheck.iFell == 2 && (theFieldBall.positionOnField.y() < theFieldDimensions.yPosLeftGoal && theFieldBall.positionOnField.y() > theFieldDimensions.yPosRightGoal))
        //   goto centralFallen;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(hayObstaculoCerca)
          goto ObsAvoid;
      }
      action
      {
        theLookForwardSkill();
        thePathToTargetSkill(1.0, Defender1Pos);
      }
    }



    state(kickRight)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        // if(theLibCheck.iFell == 2 && (theFieldBall.positionOnField.y() < theFieldDimensions.yPosLeftGoal && theFieldBall.positionOnField.y() > theFieldDimensions.yPosRightGoal))
        //   goto centralFallen;
        if(theRobotPose.translation.y() <= theFieldDimensions.yPosLeftGoal)
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theSaySkill("kick right");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() - 2000));
        if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal && theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));
      }
    }

    state(kickLeft)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        // if(theLibCheck.iFell == 2 && (theFieldBall.positionOnField.y() < theFieldDimensions.yPosLeftGoal && theFieldBall.positionOnField.y() > theFieldDimensions.yPosRightGoal))
        //   goto centralFallen;
        if(theRobotPose.translation.y() <= theFieldDimensions.yPosLeftGoal)
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theSaySkill("kick left");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,0.f,theRobotPose.inversePose.translation.y() + 2000));
        if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal && theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));
      }
    }

    state(avanceConBalon)
    {
      const Angle angleToGoal = calcAngleToGoal();
      bool hayObstaculos = hayObstaculo();

      transition
      {
        // if(theLibCheck.iFell == 2 && (theFieldBall.positionOnField.y() < theFieldDimensions.yPosLeftGoal && theFieldBall.positionOnField.y() > theFieldDimensions.yPosRightGoal))
        //   goto centralFallen;
        if(theRobotPose.translation.y() <= theFieldDimensions.yPosLeftGoal)
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
        theSaySkill("go go go");
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 2000.f, 0));
        // theInWalkKickSkill(WalkKickVariant(WalkKicks::none, Legs::left), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal && theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine){
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));
        }
      }
    }

    state(alignToPass)
    {
      const Angle angleToTeammate = calcAngleToTeammate();
      const Angle angleToGo = calcAngleToGo();

      transition
      {
        // if(theLibCheck.iFell == 2 && (theFieldBall.positionOnField.y() < theFieldDimensions.yPosLeftGoal && theFieldBall.positionOnField.y() > theFieldDimensions.yPosRightGoal))
        //   goto centralFallen;
        if(theRobotPose.translation.y() <= theFieldDimensions.yPosLeftGoal)
          goto waitBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto GiraCabezaDer; 
        if(std::abs(angleToGo) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
          goto alignBehindBallToPass;
      }

      action
      {
        theSaySkill("Align Go go");
        theLookForwardSkill();
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGo, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
        if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal && theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));
      }
    }

    state(searchForBall)
    {
      
      transition
      {
        // if(theLibCheck.iFell == 2 && (theFieldBall.positionOnField.y() < theFieldDimensions.yPosLeftGoal && theFieldBall.positionOnField.y() > theFieldDimensions.yPosRightGoal) && theFieldBall.ballWasSeen())
        //   goto centralFallen;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(!theFieldBall.ballWasSeen(10000))
          goto goBackHome;  
      }

      action
      {
        theSaySkill("search");
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
        if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal && theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));
        
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
        if(theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal && theRobotPose.translation.x() < theFieldDimensions.xPosHalfWayLine)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldDimensions.yPosLeftGoal-500));
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
        if(!theFieldBall.ballWasSeen(10000))
          goto goBackHome; 
      }

      action
      {

        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
        theLookAtAnglesSkill(-1,2);
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
        if(!theFieldBall.ballWasSeen(10000))
          goto goBackHome; 
      }
      action
      {
          theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
          theLookAtAnglesSkill(1,2);
          //a+=1;   
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
    if(theRobotPose.translation.y() > (theFieldDimensions.yPosLeftFieldBorder - 2000.f))
      random = 0;
    if(theRobotPose.translation.y() < (theFieldDimensions.yPosRightFieldBorder + 2000.f))
      random = 1;

    return random;
  }
};

MAKE_CARD(LeftDefenderCard);