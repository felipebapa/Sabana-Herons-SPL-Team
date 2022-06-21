
#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Representations/Communication/RobotInfo.h"

CARD(LookAroundCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(LookAtAngles),
  REQUIRES(RobotInfo),
  DEFINES_PARAMETERS(
  {,
    (int)(700) timeTest,
    (int)(500) initialWaitTime,
  }),
});

class LookAroundCard : public LookAroundCardBase
{
  bool preconditions() const override
  {
    return theRobotInfo.number == 6;
  }

  bool postconditions() const override
  {
    return theRobotInfo.number != 6;
  }

  option
  {

    theActivitySkill(BehaviorStatus::LookAround);
    static bool direction;

    initial_state(lookFront)
    {
        transition
        {
            if(state_time > initialWaitTime)
                goto lookDiagonalRight;
            else
                goto lookDiagonalLeft;
        }
        action
        {
            theLookForwardSkill();
            theStandSkill();
        }
    }

    state(lookDiagonalRight)
    {
        transition
        {
            if(state_time > initialWaitTime) {
                if(!direction)
                    goto lookFront;
                else
                    goto lookRight;
            }
        }
        action
        {
            theLookAtAnglesSkill(pi/6, 0, 1, HeadMotionRequest::autoCamera);
        }
    }

    state(lookDiagonalLeft)
    {
        transition
        {
            if(state_time > initialWaitTime) {
                if(direction)
                    goto lookFront;
                else
                    goto lookLeft;
            }
        }
        action
        {
            theLookAtAnglesSkill(-pi/6, 0, 1, HeadMotionRequest::autoCamera);
        }
    }

    state(lookRight) 
    {
        transition
        {
            if(state_time > initialWaitTime) {
                direction = false;
                goto lookDiagonalRight;
            }
        }
        action
        {
            theLookAtAnglesSkill(pi/3, 0, 1, HeadMotionRequest::autoCamera);
        }
    }

    state(lookLeft)
    {
        transition
        {
            if(state_time > initialWaitTime) {
                direction = true;
                goto lookDiagonalLeft;
            }
        }
        action
        {
            theLookAtAnglesSkill(-pi/3, 0, 1, HeadMotionRequest::autoCamera);
        }
    }
  }
};

MAKE_CARD(LookAroundCard);