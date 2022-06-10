/**
 * @file LibCheckProvider.cpp
 *
 * Performs some checks of the behavior control module such as
 * - How often is a MotionRequest set?
 * - How often is a HeadMotionRequest set?
 *
 * @author Tim Laue
 */

#include "LibCheckProvider.h"

MAKE_MODULE(LibCheckProvider, behaviorControl);

void LibCheckProvider::update(LibCheck& libCheck)
{
  reset();
  libCheck.inc = [this](LibCheck::CheckedOutput outputToCheck) {inc(outputToCheck);};
  libCheck.setArm = [this](Arms::Arm arm)
  {
    setArmsInThisFrame[arm] = true;
  };
  libCheck.wasSetArm = [this](Arms::Arm arm) -> bool
  {
    return setArmsInThisFrame[arm];
  };
  libCheck.performCheck = [this](const MotionRequest& theMotionRequest)
  {
    checkOutputs(theActivationGraph, static_cast<LibCheck::CheckedOutput>(0), LibCheck::firstTeamCheckedOutput);
    checkMotionRequest(theActivationGraph, theMotionRequest);
  };
  libCheck.performTeamCheck = [this]()
  {
    checkOutputs(theTeamActivationGraph, LibCheck::firstTeamCheckedOutput, LibCheck::numOfCheckedOutputs);
  };
  
  libCheck.closerToTheBall= isCloserToTheBall();
}

void LibCheckProvider::reset()
{
  FOREACH_ENUM(LibCheck::CheckedOutput, i)
    callCounters[i] = 0;

  for(int i = 0; i < Arms::numOfArms; ++i)
    setArmsInThisFrame[i] = false;
}

void LibCheckProvider::checkOutputs(const ActivationGraph& activationGraph, LibCheck::CheckedOutput start, LibCheck::CheckedOutput end) const
{
  const std::string options = getActivationGraphString(activationGraph);

  // Output counting checks:
  for(LibCheck::CheckedOutput i = start; i < end; i = static_cast<LibCheck::CheckedOutput>(static_cast<unsigned>(i) + 1))
  {
    // Check, if output has been set at least once:
    if(callCounters[i] == 0 && notSetCheck[i] == 1)
    {
      OUTPUT_TEXT("Meeek! " << TypeRegistry::getEnumName(i) << " has not been set in this cycle (Robot " << theRobotInfo.number
                  << (!callCounters[LibCheck::role] ? "" : ", Role: " + theTeamBehaviorStatus.role.getName())
                  << (options == "" ? "" : ", Options: " + options) << ") !");
    }
    else if(notSetCheck[i] == 2)
    {
      ASSERT(callCounters[i] > 0);
    }
  }
}


void LibCheckProvider::checkMotionRequest(const ActivationGraph& activationGraph, const MotionRequest& theMotionRequest) const
{
  // Check for invalid motion request:
  if(assertValidWalkRequest &&
     theMotionRequest.motion == MotionRequest::walk &&
     !theMotionRequest.walkRequest.isValid())
  {
#ifndef NDEBUG
    {
      std::string logDir = "";
#ifdef TARGET_ROBOT
      logDir = "../logs/";
#endif
      OutMapFile stream(logDir + "walkRequest.log");
      stream << theMotionRequest.walkRequest;
      stream << getActivationGraphString(activationGraph);
    }
#endif
    FAIL("Motion request is not valid (see walkRequest.log).");
  }
}

void LibCheckProvider::inc(LibCheck::CheckedOutput outputToCheck)
{
  const int index = static_cast<int>(outputToCheck);
  if(index >= 0 && index < LibCheck::numOfCheckedOutputs)
  {
    ++callCounters[index];

    // Check, if output has not been set more than once:
    if(callCounters[index] > 1)
    {
      if(multipleSetCheck[index] == 1)
      {
        const std::string options = getActivationGraphString(index >= LibCheck::firstTeamCheckedOutput ? theTeamActivationGraph : theActivationGraph);

        OUTPUT_TEXT("Meeek! " << TypeRegistry::getEnumName(static_cast<LibCheck::CheckedOutput>(index)) << " has been set more than once in this cycle (Robot "
                    << theRobotInfo.number
                    << (!callCounters[LibCheck::role] ? "" : ", Role: " + theTeamBehaviorStatus.role.getName())
                    << (options == "" ? "" : ", Options: " + options) << ") !");
      }
      else if(multipleSetCheck[index] == 2)
      {
        ASSERT(callCounters[index] <= 1);
      }
    }
  }
}

std::string LibCheckProvider::getActivationGraphString(const ActivationGraph& activationGraph) const
{
  std::string options = "";
  for(const auto& node : activationGraph.graph)
    options += (options == "" ? "" : ", ") + node.option + (node.state == "" ? "" : "/" + node.state);
  return options;
}

bool LibCheckProvider::isCloserToTheBall()
{

  double teammateDistanceToBall = 0.0;

  distanceToBall= (theRobotPose.inversePose*theTeamBallModel.position).norm();

  if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen)<=4000){
    distanceToBall= theBallModel.estimate.position.norm();
  }


  for(auto const& teammate : theTeamData.teammates)
  {
    if(!teammate.isPenalized){
      teammateDistanceToBall = (teammate.theRobotPose.inversePose*theTeamBallModel.position).norm();

      if(theFrameInfo.getTimeSince(teammate.theBallModel.timeWhenLastSeen)<=4000){
       teammateDistanceToBall = teammate.theBallModel.estimate.position.norm();
      }

      if(distanceToBall > teammateDistanceToBall)
      {
        return false;
      }
    }
  }
  return true;  //Si esto es true, el local es quien va al balòn.
}

Pose2f LibCheckProvider::teammateToPass()
{
  double teammateDistanceToBall = 0.0;
  Pose2f teammatePos;
  int counter = 0;

  for(auto const& teammate : theTeamData.teammates)
  {
    counter++;
  }

  if(counter == 0)
    return Pose2f(0,-2500,-1500);
  else if(counter == 1)
    return Pose2f(0,-5000,0); 
  else
    return Pose2f(0,0,0);
}

