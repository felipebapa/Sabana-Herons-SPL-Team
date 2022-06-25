/**
 * @file Striker.cpp
 *
 * 
 *
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
#include "Representations/Communication/TeamData.h"
#include "Modules/Communication/TeamMessageHandler/TeamMessageHandler.h"
//#include "Representations/Communication/BHumanMessage.h"




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
  REQUIRES(TeamData),
  //REQUIRES(BHumanMessage),
  
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
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        //if(theFieldBall.positionOnField.x() <= 0.f) 
          //goto goToPass; 
        if(std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold || theFieldBall.positionOnField.x() > 0.f)
          goto walkToBall;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
       // TeamMessageHandler::sendInterval;
        
       
        
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
          {
            
          if(!theObstacleModel.obstacles.empty()){     //Tenemos obstàculos, entonces, actuamos.   
            for(const auto& obstacle : theObstacleModel.obstacles){
              if ((obstacle.center.norm() < (theFieldDimensions.xPosOpponentGoal - theFieldBall.positionOnField.x())))
                goto moveSide;
              if (std::abs(obstacle.center.y()) > 150.f)
                goto longKick;
              }
              }
              else
                goto longKick;    
          }
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX-20.f, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }
    state(moveSide)
    {
      //const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if((theFieldBall.positionRelative.y() > 100.f && theFieldBall.positionRelative.x() < 20.f) || state_time > maxKickWaitTime)
          goto kickLeft;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
      }
      action
      {
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldBall.positionRelative.y() - 110.f));
        if(theFieldBall.positionRelative.y() > 95.f)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theFieldBall.positionRelative.x() - 100.f, 0.f));
      }
    }

    state(kickLeft)
    {
      //const Angle angleToGoal = calcAngleToGoal();
      
      transition
      {
        if(state_time > maxKickWaitTime || theKickSkill.isDone())
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theKickSkill((KickRequest::SideKick), true, 0.3f, false);
        //theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        
      }
    }
    state(longKick)
    {     
      transition
      {
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theKickSkill((KickRequest::kickForward), true, 0.3f, false);
      }
    }

    state(searchForBall)
    {
      transition
      {
        if(state_time > 1500)
          goto lookLeft;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
      }

      action
      {
        theLookForwardSkill();
        theLookAtAnglesSkill(-1,2);
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }
    state(lookLeft)
    {
      transition
      {
        if(state_time > 1500)
          goto searchForBall;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
      }
      action
      {
        theLookForwardSkill();
        theLookAtAnglesSkill(1,2);
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));

      }
    }
    state(goToPass)
    {
      transition
      {
        if(theLibCheck.positionToPass)
          goto prueba;
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;        
      }
      action
      {
        theSaySkill("Go to pass");
        thePathToTargetSkill(1.0,Pose2f(pi,500.f,1000.f));
      }
    }

    state(prueba)
    {
      transition
      {
        if(!theLibCheck.positionToPass)
          goto lookLeft;
        if(theFieldBall.ballWasSeen())
          goto turnToBall; 
      }
      action
      {
        theSaySkill("yeesss");
        theKeyFrameArmsSkill(ArmKeyFrameRequest::back,false);
      }
    }
  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }  

};

MAKE_CARD(StrikerCard);
