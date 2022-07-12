/**
 * @file Keeper.cpp
 *
 * Pruebas
 *
 * @author Felipe Jose Santiago
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
     {
         ,
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
         CALLS(Say),
         CALLS(Kick),
         CALLS(LookAtAngles),

         REQUIRES(FieldBall),
         REQUIRES(BallModel),
         REQUIRES(FieldDimensions),
         REQUIRES(RobotPose),
         REQUIRES(RobotInfo),
         DEFINES_PARAMETERS(
             {
                 ,
                 (float)(0.8f)walkSpeed,
                 (int)(1000)initialWaitTime,
                 (int)(7000)ballNotSeenTimeout,
                 (Angle)(7_deg)ballAlignThreshold,
                 (float)(500.f)ballNearThreshold,
                 (Angle)(10_deg)angleToGoalThreshold,
                 (float)(400.f)ballAlignOffsetX,
                 (float)(100.f)ballYThreshold,
                 (float)(100.f)ballXThreshold,
                 (Angle)(2_deg)angleToGoalThresholdPrecise,
                 (float)(150.f)ballOffsetX,
                 (Rangef)({140.f, 170.f})ballOffsetXRange,
                 (float)(40.f)ballOffsetY,
                 (Rangef)({20.f, 50.f})ballOffsetYRange,
                 (int)(10)minKickWaitTime,
                 (int)(3000)maxKickWaitTime,
                 (Pose2f)(Pose2f(0, -4100, 0))KeeperPos,
                 (int)(100)StopThreshold,
                 (float)(15_deg)AngleThreshold,
                 (float)(25_deg)AngleThresholdSearch,
                 (Angle)(180_deg) angleOffset,
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
        if (state_time > initialWaitTime)
          goto goBackHome;
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }
    state(goBackHome)
    {
      transition
      {
        if (theRobotPose.translation.x() > theFieldDimensions.xPosOwnGroundline + 100 && std::abs(theRobotPose.translation.y()) < 10 && std::abs(theRobotPose.inversePose.translation.angle()) < 5_deg)
          goto searchForBall;
      }
      action
      {
        theLookForwardSkill();
        thePathToTargetSkill(1.0f, KeeperPos);
      }
    }
    state(checkBall)
    {
      transition
      {
        if (!theFieldBall.ballWasSeen(7000))
          goto searchForBall;
        if (theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
          goto defensive;
        if (theRobotPose.translation.x() >= theFieldDimensions.xPosOwnGroundline + 1700 && theFieldBall.positionOnField.x() > theFieldDimensions.xPosHalfWayLine && theFieldBall.positionRelative.norm() > 1000)
          goto goBackHome;
        if (theFieldBall.positionOnField.x() >= theFieldDimensions.xPosHalfWayLine)
          goto lookout;
      }
      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }
    state(searchForBall)
    {
      transition
      {
        if (theFieldBall.ballWasSeen())
          goto checkBall;
        if (state_time > 2000 && !theFieldBall.ballWasSeen(3000))
          goto lookRight;
        if (std::abs(theRobotPose.translation.angle()) > 30_deg && theRobotPose.translation.x() > -2500 && theFieldBall.positionRelative.norm() > 1000)
          goto goBackHome;
      }
      action
      {
        theLookAtAnglesSkill(-1, 1.5f, 0.8f);
        theStandSkill();
      }
    }
    state(lookRight)
    {
      transition
      {
        if (theFieldBall.ballWasSeen())
          goto checkBall;
        if (state_time > 2000 && !theFieldBall.ballWasSeen(3000))
          goto searchForBall;
        if (theRobotPose.translation.x() > -2500 && theFieldBall.positionRelative.norm() > 1000)
          goto goBackHome;
      }
      action
      {
        theLookAtAnglesSkill(1, 1.5f, 0.8f);
        //theStandSkill();
      }
    }
    state(defensive)
    {
      transition
      {
        if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(!theFieldBall.ballWasSeen(6000) && theRobotPose.translation.x() >= theFieldDimensions.xPosOwnPenaltyArea)
          goto goBackHome;
        if ((theFieldBall.positionOnField.x() < theFieldDimensions.xPosOwnPenaltyArea && (theFieldDimensions.yPosLeftPenaltyArea) > theFieldBall.positionOnField.y() > (theFieldDimensions.yPosLeftPenaltyArea)) || theFieldBall.positionRelative.norm() <= 1400)
          goto despeje;

        if (theFieldBall.positionOnField.y() > theFieldDimensions.yPosCenterGoal + 600 && theRobotPose.translation.y() > 605 /*(theFieldDimensions.yPosLeftGoal - 200) */) // Goes over left
          goto posLeft;
        if (theFieldBall.positionOnField.y() < theFieldDimensions.yPosCenterGoal - 600 && theRobotPose.translation.y() < -605/* theRobotPose.translation.y() > 370 (theFieldDimensions.yPosLeftGoal - 200) */) // Goes over left
          goto posRight;
        if(std::abs(theFieldBall.positionOnField.y()) <= 600)
          goto posMid;
        if (theFieldBall.positionOnField.x() >= theFieldDimensions.xPosHalfWayLine)
          goto lookout;
      }
      action
      {
        theLookAtAnglesSkill(theFieldBall.positionRelative.angle(), 2);
        
      }
    }
    state(posRight)
    {
      const Angle angleToGoal = calcAngleToGoal();
      transition
      {
        if (theFieldBall.positionOnField.x() >= theFieldDimensions.xPosHalfWayLine || theFieldBall.positionOnField.y() >= theFieldDimensions.yPosCenterGoal - 600)
          goto defensive;
        if ((theFieldBall.positionOnField.x() < theFieldDimensions.xPosOwnPenaltyArea && (theFieldDimensions.yPosLeftPenaltyArea) > theFieldBall.positionOnField.y() > (theFieldDimensions.yPosLeftPenaltyArea)) || theFieldBall.positionRelative.norm() <= 1400)
          goto despeje;
        if(!theFieldBall.ballWasSeen(6000))
          goto searchForBall;

        if (100 > theBallModel.estimate.position.y() && theFieldBall.endPositionRelative.x() < 0 && theBallModel.estimate.velocity.x() < -90)
          goto GoalRiskRight;
        if (-100 < theBallModel.estimate.position.y()  && theFieldBall.endPositionRelative.x() < 0 && theBallModel.estimate.velocity.x() < -90)
          goto GoalRiskLeft;
        if(-100 < theBallModel.estimate.position.y() && theBallModel.estimate.position.y() < 100 && theFieldBall.endPositionRelative.x() < 0 && theBallModel.estimate.velocity.x() < -90)
          goto goDown;
      }
      action
      {
        if(theRobotPose.translation.y() < -450)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, 0.f, 200));
        if(theRobotPose.translation.x() < theFieldDimensions.xPosOwnGroundline + 150 )
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 200, 0.f));
        if(std::abs (theRobotPose.translation.y()) <= 450)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldBall.positionRelative.y()));
        theLookAtAnglesSkill(theFieldBall.positionRelative.angle(), 2);
      }
    }
    state(posLeft)
    {
      const Angle angleToGoal = calcAngleToGoal();
      transition
      {
        if (theFieldBall.positionOnField.x() >= theFieldDimensions.xPosHalfWayLine || theFieldBall.positionOnField.y() <= theFieldDimensions.yPosCenterGoal + 600)
          goto defensive;
        if ((theFieldBall.positionOnField.x() < theFieldDimensions.xPosOwnPenaltyArea && (theFieldDimensions.yPosLeftPenaltyArea) > theFieldBall.positionOnField.y() > (theFieldDimensions.yPosLeftPenaltyArea)) || theFieldBall.positionRelative.norm() <= 1400)
          goto despeje;
        if(!theFieldBall.ballWasSeen(6000))
          goto searchForBall;
        
        if (100 > theBallModel.estimate.position.y() && theFieldBall.endPositionRelative.x() < 0 && theBallModel.estimate.velocity.x() < -90)
          goto GoalRiskRight;
        if (-100 < theBallModel.estimate.position.y()  && theFieldBall.endPositionRelative.x() < 0 && theBallModel.estimate.velocity.x() < -90)
          goto GoalRiskLeft;
        if(-100 < theBallModel.estimate.position.y() && theBallModel.estimate.position.y() < 100 && theFieldBall.endPositionRelative.x() < 0 && theBallModel.estimate.velocity.x() < -90)
          goto goDown;
      }
      action
      {
        if(theRobotPose.translation.y() > 450)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, 0.f, -200));
        if(theRobotPose.translation.x() < theFieldDimensions.xPosOwnGroundline + 150 )
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 200, 0.f));
        if(std::abs (theRobotPose.translation.y()) <= 450)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldBall.positionRelative.y()));
        theLookAtAnglesSkill(theFieldBall.positionRelative.angle(), 2);
      }
    }
    state(posMid)
    {
      transition
      {
        if (theFieldBall.positionOnField.x() >= theFieldDimensions.xPosHalfWayLine || theFieldBall.positionOnField.y() < -600 || theFieldBall.positionOnField.y() > 600 )
          goto defensive;
        if ((theFieldBall.positionOnField.x() < theFieldDimensions.xPosOwnPenaltyArea && (theFieldDimensions.yPosLeftPenaltyArea) > theFieldBall.positionOnField.y() > (theFieldDimensions.yPosLeftPenaltyArea)) || theFieldBall.positionRelative.norm() <= 1400)
          goto despeje;
        if(!theFieldBall.ballWasSeen(6000))
          goto searchForBall;
          
        if (100 > theBallModel.estimate.position.y() && theFieldBall.endPositionRelative.x() < 0 && theBallModel.estimate.velocity.x() < -90)
          goto GoalRiskRight;
        if (-100 < theBallModel.estimate.position.y()  && theFieldBall.endPositionRelative.x() < 0 && theBallModel.estimate.velocity.x() < -90)
          goto GoalRiskLeft;
        if(-100 < theBallModel.estimate.position.y() && theBallModel.estimate.position.y() < 100 && theFieldBall.endPositionRelative.x() < 0 && theBallModel.estimate.velocity.x() < -90)
          goto goDown;
      }
      action
      {
        if(std::abs (theRobotPose.translation.y()) <= 450)
        {
          if(theFieldBall.positionRelative.y() < 0)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, -300.f));
          if(theFieldBall.positionRelative.y() > 0)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, 300.01f));
        }
        if(theRobotPose.translation.y() > 450)
          theWalkToTargetSkill(Pose2f(walkSpeed,walkSpeed, walkSpeed), Pose2f(0.f, 0.f, - 200));
        if(theRobotPose.translation.y() < -450)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, 200));
        if(theRobotPose.translation.x() < theFieldDimensions.xPosOwnGroundline + 150 )
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 200, 0.f));
        theLookAtAnglesSkill(theFieldBall.positionRelative.angle(), 1.7f);
      }
    }
    state(GoalRiskRight)
    {
      transition
      {
        if (!theFieldBall.ballWasSeen(ballNotSeenTimeout) || theSpecialActionSkill.isDone() || state_time > 5000)
          goto searchForBall;
        // CAMBIAR POR SPECIAL ACTION
      }
      action
      {
        //theSaySkill("GR R"); // CAMBIAR POR SPECIAL ACTION
        theLookForwardSkill();
        theSpecialActionSkill(SpecialActionRequest::rightDive);
      }
    }
    state(GoalRiskLeft)
    {
      transition
      {
        if (!theFieldBall.ballWasSeen(ballNotSeenTimeout) || theSpecialActionSkill.isDone() || state_time > 5000)
          goto searchForBall;
        // CAMBIAR POR SPECIAL ACTION
      }
      action
      {
        //theSaySkill("GR L"); // CAMBIAR POR SPECIAL ACTION
        theLookForwardSkill();
        theSpecialActionSkill(SpecialActionRequest::leftDive);
        
      }
    }
    state(goDown)
    {
      transition
      {
        if (!theFieldBall.ballWasSeen(ballNotSeenTimeout) || theSpecialActionSkill.isDone() || state_time > 5000)
          goto searchForBall;

        // CAMBIAR POR SPECIAL ACTION
      }
      action
      {
        //theSaySkill("Go Down"); // CAMBIAR POR SPECIAL ACTION
        theLookForwardSkill();
        theSpecialActionSkill(SpecialActionRequest::preventBall);
        
      }
    }
    state(despeje)
    {
      transition
      {
        if (theRobotPose.translation.x() > theFieldDimensions.xPosOwnPenaltyArea && theFieldBall.positionRelative.norm() > 1500)
          goto goBackHome;
        if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if (theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold) && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
          goto kick;
        if ((theFieldBall.positionOnField.x() > theFieldDimensions.xPosOwnPenaltyArea + 500 || (theRobotPose.translation.x() >= theFieldDimensions.xPosOwnPenaltyArea + 300 && !theFieldBall.ballWasSeen())))
          goto defensive;
        
        if (-100 > theFieldBall.positionRelative.y() && theFieldBall.endPositionRelative.x() < 0 && theBallModel.estimate.velocity.x() < -90)
          goto GoalRiskRight;
        if (100 < theFieldBall.positionRelative.y()  && theFieldBall.endPositionRelative.x() < 0 && theBallModel.estimate.velocity.x() < -90)
          goto GoalRiskLeft;
      }

      action
      {
        theSaySkill("Desp");
        theLookForwardSkill();
        if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosOpponentPenaltyArea)
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        else
          theLookAtAnglesSkill(theFieldBall.positionRelative.angle(), 1.7f);
      }
    }
    state(kick)
    {
      const Angle angleToGoal = calcAngleToGoal();
      transition
      {
        if (state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto checkBall;
      }

      action
      {
        theLookForwardSkill();
        theInWalkKickSkill(WalkKickVariant(WalkKicks::forwardOLD, Legs::left), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }
    state(lookout)
    {
      transition
      {
        if (theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
          goto checkBall;
        if (!theFieldBall.ballWasSeen(2000) && state_time > 3000)
          goto lookoutRight;
        if (theFieldBall.ballWasSeen())
          goto stare;
        if (std::abs(theRobotPose.translation.angle()) > 30_deg && !theFieldBall.ballWasSeen(3000))
          goto goBackHome;
      }
      action
      {
        theStandSkill();
        theLookAtAnglesSkill(-1, 1.3f, 0.8f);
      }
    }
    state(lookoutRight)
    {
      transition
      {
        if (theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
          goto checkBall;
        if (!theFieldBall.ballWasSeen(2000) && state_time > 3000)
          goto lookout;
        if (theFieldBall.ballWasSeen())
          goto stare;
        if (std::abs(theRobotPose.translation.angle()) > 30_deg && !theFieldBall.ballWasSeen(3000))
          goto goBackHome;
      }
      action
      {
        theStandSkill();
        theLookAtAnglesSkill(1, 1.3f, 0.8f);
      }
    }
    state(stare)
    {
      transition
      {
        if (!theFieldBall.ballWasSeen(3000))
          goto lookout;
        if (std::abs(theRobotPose.translation.angle()) > 30_deg && !theFieldBall.ballWasSeen(7000))
          goto goBackHome;
        if(theFieldBall.positionOnField.x() <= theFieldDimensions.xPosHalfWayLine)
          goto defensive;
      }
      action
      {
        theLookAtAnglesSkill(theFieldBall.positionRelative.angle(), 2);
      }
    }
  }
  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }
  Angle calcAngleToBall() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldBall.positionOnField.x(), theFieldBall.positionOnField.y())).angle();
  }
  Angle calcAngleOwnGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle();
  }
};
MAKE_CARD(KeeperCard);