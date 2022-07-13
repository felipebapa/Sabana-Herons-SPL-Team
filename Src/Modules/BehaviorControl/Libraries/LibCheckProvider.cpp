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
  
  libCheck.positionToPass = positionToPass();
  libCheck.positionToPassLeft = positionToPassLeft();
  libCheck.positionToPassRight = positionToPassRight();
  libCheck.closerToTheBall= isCloserToTheBall();
  libCheck.LeftAttacking= isLeftAttacking();
  libCheck.LeftDefending= isLeftDefending();
  libCheck.RightAttacking= isRightAttacking();
  libCheck.RightDefending= isRightDefending();
  libCheck.CentralDefending = isCentralDefending();
  libCheck.StrikerAttacking = isStrikerAttacking();
  libCheck.TeammateFallenNumber=isTeammateFallenNumber();
  libCheck.TeammateObstacleAvoid=isTeammateObstacleAvoid();
  libCheck.OpponentObstacle=isOpponentObstacle();
  libCheck.TeammateSeeingBall=isTeammateSeeingBall();
  libCheck.centralLeave = centralLeave();
  libCheck.leftLeave = leftLeave();
  libCheck.rightLeave = rightLeave();
  libCheck.leftEnter = leftEnter();
  libCheck.rightEnter = rightEnter();
  libCheck.centralPenalized = isCentralPenalized();
  libCheck.rightPenalized = isRightPenalized();
  libCheck.leftPenalized = isLeftPenalized();
  libCheck.howManyPenalized = howManyPenalized();
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

bool LibCheckProvider::positionToPass()
{
  bool IsToPass = false;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(!teammate.isPenalized){
      if(teammate.number == 4 && teammate.theRobotPose.translation.x() >= 1500 && teammate.theRobotPose.translation.x() < 2000 && teammate.theRobotPose.translation.y() >= 500 && teammate.theRobotPose.translation.y() < 1500)
        IsToPass = true;
    }
  }
  return IsToPass;
}

bool LibCheckProvider::positionToPassLeft()
{
  bool IsToPass = false;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(!teammate.isPenalized){
      if(teammate.number == 4 && teammate.theRobotPose.translation.x() >= 2500 && teammate.theRobotPose.translation.x() < 3500 && teammate.theRobotPose.translation.y() >= 1000 && teammate.theRobotPose.translation.y() < 2000)
        IsToPass = true;
    }
  }
  return IsToPass;
}

bool LibCheckProvider::positionToPassRight()
{
  bool IsToPass = false;
  for(auto const& teammate : theTeamData.teammates)
  {
    if(!teammate.isPenalized){
      if(teammate.number == 4 && teammate.theRobotPose.translation.x() >= 2500 && teammate.theRobotPose.translation.x() < 3500 && teammate.theRobotPose.translation.y() <= -1000 && teammate.theRobotPose.translation.y() > -2000)
        IsToPass = true;
    }
  }
  return IsToPass;
}
bool LibCheckProvider::isLeftAttacking()
{
  for(auto const& teammate : theTeamData.teammates)
  {
      if(!teammate.isPenalized){
        // if(teammate.theTeamBehaviorStatus.role.playBall){
          if(teammate.number==3 && teammate.theRobotPose.translation.x()>= theFieldDimensions.xPosHalfWayLine){
            return true;   //El left supporter està atacando.
        }
      } 
    }
    return false;
}

bool LibCheckProvider::isLeftDefending()
{
  for(auto const& teammate : theTeamData.teammates)
  {
    if(!teammate.isPenalized){
      if(teammate.number==3 && teammate.theRobotPose.translation.x() <= theFieldDimensions.xPosHalfWayLine && LibCheckProvider::isCloserToTheBall()==teammate.number){
        return true;
      }
    } 
  }
  return false;
}

int LibCheckProvider::isCloserToTheBall()
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
        return teammate.number;
      }
    }
  }
  return theRobotInfo.number;  //Devuelve el # de robot que està mas cerca al balòn.
}

bool LibCheckProvider::isRightAttacking()
{
  for(auto const& teammate : theTeamData.teammates)
  {
    if(!teammate.isPenalized){
      if(teammate.number==5 && teammate.theRobotPose.translation.x()>=theFieldDimensions.xPosHalfWayLine)
        return true;   //El Right supporter està atacando.
    }
  }
  return false;
}

bool LibCheckProvider::isRightDefending()
{
    for(auto const& teammate : theTeamData.teammates)
  {

      if(!teammate.isPenalized){
        // if(teammate.theTeamBehaviorStatus.role.playBall)
          if(teammate.number==5 && teammate.theRobotPose.translation.x() <= theFieldDimensions.xPosHalfWayLine && LibCheckProvider::isCloserToTheBall()==teammate.number)
            return true;   //El Right supporter està atacando.
        }
    }
    return false;
}

bool LibCheckProvider::isCentralDefending()
{
  for(auto const& teammate : theTeamData.teammates)
  {
      if(!teammate.isPenalized){
        // if(teammate.theTeamBehaviorStatus.role.playBall)
          if(teammate.number==2 && teammate.theRobotPose.translation.x() <= theFieldDimensions.xPosHalfWayLine && LibCheckProvider::isCloserToTheBall()==teammate.number)
            return true;   //El Right supporter està atacando.
        }
    }
    return false;
}

bool LibCheckProvider::isStrikerAttacking()
{
  for(auto const& teammate : theTeamData.teammates)
  {
      if(!teammate.isPenalized){
        // if(teammate.theTeamBehaviorStatus.role.playBall)
          if(teammate.number==4 && teammate.theRobotPose.translation.x() >= theFieldDimensions.xPosHalfWayLine && LibCheckProvider::isCloserToTheBall()==teammate.number)
            return true;   //El Right supporter està atacando.
        }
    }
    return false;
}

int LibCheckProvider::isTeammateFallenNumber()
{
  for(auto const& teammate : theTeamData.teammates)
  {
      if(!teammate.isPenalized)
        if(teammate.status==Teammate::FALLEN)
          return teammate.number;

    }
    return 0;
}


bool LibCheckProvider::isTeammateObstacleAvoid()
{

      if(!theObstacleModel.obstacles.empty()){
      for(const auto& obstacle : theObstacleModel.obstacles){



      if (obstacle.type == Obstacle::teammate) { 

          return true;

      }

      }
    }
    return false;
}


bool LibCheckProvider::isOpponentObstacle()
{

      if(!theObstacleModel.obstacles.empty()){
      for(const auto& obstacle : theObstacleModel.obstacles){



      if (obstacle.type == Obstacle::opponent) { 

          return true;

      }

      }
    }
    return false;
}


bool LibCheckProvider::isTeammateSeeingBall()  // Para el kickoff opponent.
{

      if(theTeamBallModel.velocity.norm()!=0){
          return true;
    }
    return false;
}

bool LibCheckProvider::isCentralPenalized()
{
  bool isPenalized = false;

  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.isPenalized && teammate.number == 2)
      isPenalized = true;
  }
  return isPenalized;
}

bool LibCheckProvider::isLeftPenalized()
{
  bool isPenalized = false;

  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.isPenalized && teammate.number == 3)
      isPenalized = true;
  }
  return isPenalized;
}

bool LibCheckProvider::isRightPenalized()
{
  bool isPenalized = false;

  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.isPenalized && teammate.number == 5)
      isPenalized = true;
  }
  return isPenalized;
}

int LibCheckProvider::howManyPenalized()
{
  int counter = 0;

  for(auto const& teammate : theTeamData.teammates)
  {
    if(teammate.isPenalized)
      counter += 1;
  }

  return counter;
}

int LibCheckProvider::centralLeave()
{
  if(isLeftAttacking() || isRightAttacking() || (howManyPenalized() == 1 && !isCentralPenalized()))
    return 6;
  else if(isCentralPenalized() && isLeftPenalized())
    return 5;
  else if(isCentralPenalized() && isRightPenalized())
    return 3;
  else
    return 2;
}

int LibCheckProvider::rightLeave()
{
  if((isLeftAttacking() && !isRightAttacking()) || (howManyPenalized() != 0 && !isRightPenalized()))
    return 6;
  else
    return 5;
}

int LibCheckProvider::leftLeave()
{
  if((isRightAttacking() && !isLeftAttacking()) || (howManyPenalized() != 0 && !isLeftPenalized()))
    return 6;
  else 
    return 3;
}

int LibCheckProvider::leftEnter()
{
    if((theRobotInfo.number == 2 && isLeftPenalized()) || (theRobotInfo.number == 2 && isLeftAttacking() && !isRightAttacking()))
      return 2;
    else if ((theRobotInfo.number == 3 && isCentralPenalized()) || (theRobotInfo.number == 3 && isRightPenalized()) || (theRobotInfo.number == 3 && isRightAttacking()))
      return 3;
    else 
      return 6;
}

int LibCheckProvider::rightEnter()
{
    if((theRobotInfo.number == 2 && isRightPenalized()) || (theRobotInfo.number == 2 && isRightAttacking() && !isLeftAttacking()))
      return 2;
    else if ((theRobotInfo.number == 5 && isCentralPenalized()) || (theRobotInfo.number == 5 && isLeftPenalized()) || (theRobotInfo.number == 5 && isLeftAttacking()))
      return 5;
    else 
      return 6;
}