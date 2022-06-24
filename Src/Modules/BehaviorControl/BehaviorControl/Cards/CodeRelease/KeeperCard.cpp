/**
 * @file Keeper.cpp
 *
 * Pruebas
 *
 * @author DMF
 * @author Jose
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
                 (int)(3000)ballNotSeenTimeout,
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
                 (Pose2f)(Pose2f(0, -4400, 0))KeeperPos,
                 (int)(100)StopThreshold,
                 (float)(15_deg)AngleThreshold,
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
        theSaySkill("Start");
      }
    }
    state(goBackHome)
    {
      transition
      {
        if (theRobotPose.translation.x() < theFieldDimensions.xPosOwnGroundline + 150 && std::abs(theRobotPose.translation.y()) < 10 && std::abs(theRobotPose.inversePose.translation.angle()) < 5_deg)
          goto searchForBall;
      }
      action
      {
        theLookForwardSkill();
        thePathToTargetSkill(0.8f, KeeperPos);
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
        if (theRobotPose.translation.x() >= theFieldDimensions.xPosOwnGroundline + 1700 && theFieldBall.positionOnField.x() > theFieldDimensions.xPosHalfWayLine)
          goto goBackHome;
        if (theFieldBall.positionOnField.x() >= theFieldDimensions.xPosHalfWayLine)
          goto lookout;
      }
      action
      {
        theLookForwardSkill();
        theStandSkill();
        theSaySkill("Check Ball");
      }
    }
    state(searchForBall)
    {
      transition
      {
        if (theFieldBall.ballWasSeen())
          goto checkBall;
        if (state_time > 1500 && !theFieldBall.ballWasSeen(3000))
          goto lookRight;
        if (std::abs(theRobotPose.translation.angle()) > 30_deg && theRobotPose.translation.x() > -2500)
          goto goBackHome;
      }
      action
      {
        theLookAtAnglesSkill(-1, 2, 0.8f);
        theSaySkill("Search For Ball");
        theStandSkill();
      }
    }
    state(lookRight)
    {
      transition
      {
        if (theFieldBall.ballWasSeen())
          goto checkBall;
        if (state_time > 1500 && !theFieldBall.ballWasSeen(3000))
          goto searchForBall;
        if (theRobotPose.translation.x() > -2500)
          goto goBackHome;
      }
      action
      {
        theLookAtAnglesSkill(1, 2, 0.8f);
        theSaySkill("Look Right");
        theStandSkill();
      }
    }
    state(checkPos)
    {
      transition
      {
        if (theRobotPose.translation.x() < theFieldDimensions.xPosOwnPenaltyArea && std::abs(theRobotPose.translation.y()) > 1100)
          goto goBackHome;
        if (theFieldBall.ballWasSeen() && theFieldBall.positionOnField.x() < -2500)
          goto checkBall;
      }
      action
      {
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
        theSaySkill("Check Pos");
      }
    }
    state(defensive)
    {
      transition
      {
        if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if ((theFieldBall.positionOnField.x() < theFieldDimensions.xPosOwnPenaltyArea && (theFieldDimensions.yPosLeftPenaltyArea) > theFieldBall.positionOnField.y() > (theFieldDimensions.yPosLeftPenaltyArea)) || theFieldBall.positionRelative.norm() <= 1500)
          goto despeje;

        if (-100 > theBallModel.estimate.position.y() && theFieldBall.endPositionRelative.x() < theFieldDimensions.xPosOwnGroundline && theRobotPose.translation.y() > theFieldDimensions.yPosRightGoal + 200.f)
          goto GoalRiskRight;
        if (100 < theBallModel.estimate.position.y() && theFieldBall.endPositionRelative.x() < theFieldDimensions.xPosOwnGroundline && theRobotPose.translation.y() < theFieldDimensions.yPosLeftGoal - 200.f)
          goto GoalRiskLeft;

        /* if ((theFieldDimensions.xPosHalfWayLine > theFieldBall.positionOnField.x() > theFieldDimensions.xPosOwnPenaltyArea + 1050)) // Ball Outside Penalty Area, Before MidField
        {
          goto aware;
        } */
        if (theFieldBall.positionOnField.x() >= theFieldDimensions.xPosHalfWayLine)
          goto lookout;
      }
      action
      {
        const Angle angleToGoal = calcAngleToGoal();
        if (theRobotPose.translation.x() > (theFieldDimensions.xPosOwnPenaltyArea + 1050)) // Goes over penalty area
        {
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldDimensions.xPosOwnGroundline, 0.f));
          theSaySkill("Oops went over");
        }
        if (theRobotPose.translation.y() > 360 /* (theFieldDimensions.yPosLeftGoal - 200) */) // Goes over left
        {
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theRobotPose.inversePose.translation.y() - 200));
          theSaySkill("Oops went Left");
        }
        if (theRobotPose.translation.y() < -360 /* (theFieldDimensions.yPosRightGoal + 200) */) // Goes over right
        {
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theRobotPose.inversePose.translation.y() + 200));
          theSaySkill("Oops went right");
        }

        if (std::abs(theFieldBall.positionOnField.y()) <= 1500 && std::abs(theRobotPose.translation.y() <= 370))
        {
          theSaySkill("On Own Field");
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 0.f, theFieldBall.positionRelative.y()));
        } else
        {
          theStandSkill();
          theLookAtAnglesSkill(theFieldBall.positionRelative.angle(), 2);
        }
      }
    }
    state(GoalRiskRight)
    {
      transition
      {
        if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if (theSaySkill.isDone()) // CAMBIAR POR SPECIAL ACTION
          goto start;
      }
      action
      {
        theLookForwardSkill();
        theSaySkill("Right Siuuuuuu");
      }
    }
    state(GoalRiskLeft)
    {
      transition
      {
        if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if (theSaySkill.isDone()) // CAMBIAR POR SPECIAL ACTION
          goto start;
      }
      action
      {
        theLookForwardSkill();
        // theSpecialActionSkill(SpecialActionRequest::leftDive);
        theSaySkill("Left Siuuuuuu");
      }
    }
    state(despeje)
    {
      transition
      {
        if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if (theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold) && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
          goto kick;
        if ((theFieldBall.positionOnField.x() > theFieldDimensions.xPosOwnPenaltyArea + 2000.f && std::abs(theFieldBall.positionOnField.y()) >= 1100) || theFieldBall.endPositionOnField.x() <= theFieldDimensions.xPosOwnGroundline)
          goto start;
        if (theRobotPose.translation.x() > theFieldDimensions.xPosOwnPenaltyMark + 2500.f)
          goto goBackHome;
      }

      action
      {
        theLookForwardSkill();
        theSaySkill("Cleaaar");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }
    state(alignToGoal)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if (std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
          goto alignBehindBall;
        if ((theFieldBall.positionOnField.x() > theFieldDimensions.xPosOwnPenaltyMark + 1700 && std::abs(theFieldBall.positionOnField.y()) >= 1100) || theFieldBall.endPositionRelative.x() <= 0)
          goto start;

      }

      action
      {
        theLookForwardSkill();
        theSaySkill("Align to Goal");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
      }
    }
    state(alignBehindBall)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if (std::abs(angleToGoal) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
          goto kick;
        if ((theFieldBall.positionOnField.x() > theFieldDimensions.xPosOwnPenaltyMark + 1700 && std::abs(theFieldBall.positionOnField.y()) >= 1100) || theFieldBall.endPositionRelative.x() <= 0)
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theSaySkill("Align Behind Ball");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }
    state(kick)
    {
      const Angle angleToGoal = calcAngleToGoal();
      transition
      {
        if (state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theSaySkill("Kick");
        //theKickSkill((KickRequest::kickForward), true, 0.3f, false);
        theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }
    state(lookout)
    {
      const Angle angleToBall = calcAngleToBall();
      transition
      {
        if (theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
          goto checkBall;
        if (!theFieldBall.ballWasSeen(2000))
          goto lookoutRight;
        if (theFieldBall.ballWasSeen())
          goto stare;
        if (std::abs(theRobotPose.translation.angle()) > 30_deg && !theFieldBall.ballWasSeen(3000))
          goto goBackHome;
      }
      action
      {
        theStandSkill();
        theLookAtAnglesSkill(-1, 2, 0.6f);
      }
    }
    state(lookoutRight)
    {
      const Angle angleToBall = calcAngleToBall();
      transition
      {
        if (theFieldBall.positionOnField.x() < theFieldDimensions.xPosHalfWayLine)
          goto checkBall;
        if (!theFieldBall.ballWasSeen(2000))
          goto lookout;
        if (theFieldBall.ballWasSeen())
          goto stare;
        if (std::abs(theRobotPose.translation.angle()) > 30_deg && !theFieldBall.ballWasSeen(3000))
          goto goBackHome;
      }
      action
      {
        theStandSkill();
        theLookAtAnglesSkill(1, 2, 0.6f);
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
        theSaySkill("stare");
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