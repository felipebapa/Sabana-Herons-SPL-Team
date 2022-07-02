/**
 * @file TeamMessageHandler.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "TeamMessageHandler.h"
#include "Tools/MessageQueue/OutMessage.h"
#include "Tools/Global.h"
#include "Platform/Time.h"

//#define SITTING_TEST
//#define SELF_TEST

MAKE_MODULE(TeamMessageHandler, communication)

void TeamMessageHandler::update(BHumanMessageOutputGenerator& outputGenerator)
{

if(theMotionInfo.motion == MotionInfo::kick || theMotionInfo.motion == MotionInfo::specialAction || 
theBehaviorStatus.activity== BehaviorStatus::initial || theBehaviorStatus.activity== BehaviorStatus::finished || 
theBehaviorStatus.activity== BehaviorStatus::codeReleasePositionForKickOff || theBehaviorStatus.activity== BehaviorStatus::set){


    TeamMessageHandler::NoMandarMensaje();

}else if(theRawGameInfo.setPlay == SET_PLAY_KICK_IN || theRawGameInfo.setPlay == SET_PLAY_CORNER_KICK){

    TeamMessageHandler::MensajeSporadico();

}else{

    TeamMessageHandler::MandarMensaje();
 
}

  outputGenerator.theBHumanArbitraryMessage.queue.clear();

  outputGenerator.sendThisFrame =
#ifndef SITTING_TEST
#ifdef TARGET_ROBOT
    !(theMotionRequest.motion == MotionRequest::specialAction && theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::playDead) &&
    !(theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::playDead) &&
#endif
#endif // !SITTING_TEST
    (theFrameInfo.getTimeSince(timeLastSent) >= sendInterval || theFrameInfo.time < timeLastSent);

  outputGenerator.generate = [this, &outputGenerator](RoboCup::SPLStandardMessage* const m)
  {
    generateMessage(outputGenerator);
    writeMessage(outputGenerator, m);
  };
}

void TeamMessageHandler::generateMessage(BHumanMessageOutputGenerator& outputGenerator) const
{
#define SEND_PARTICLE(particle) \
  the##particle >> outputGenerator

  outputGenerator.theBSPLStandardMessage.playerNum = static_cast<uint8_t>(theRobotInfo.number);
  outputGenerator.theBSPLStandardMessage.teamNum = static_cast<uint8_t>(Global::getSettings().teamNumber);
  outputGenerator.theBHumanStandardMessage.magicNumber = Global::getSettings().magicNumber;

  outputGenerator.theBHumanStandardMessage.timestamp = Time::getCurrentSystemTime();

  outputGenerator.theBHumanStandardMessage.hasGroundContact = theGroundContactState.contact && theMotionInfo.motion != MotionInfo::getUp && theMotionRequest.motion != MotionRequest::getUp;
  outputGenerator.theBHumanStandardMessage.isUpright = theFallDownState.state == theFallDownState.upright || theFallDownState.state == FallDownState::staggering || theFallDownState.state == FallDownState::squatting;
  if(theGroundContactState.contact) //FIXME waiting for reponse of andi -> TASK: speaking with tim instead; if this is not deliberately it hast to be outputGenerator.theBHumanStandardMessage.hasGroundContact instead
    outputGenerator.theBHumanStandardMessage.timeOfLastGroundContact = theFrameInfo.time;

  outputGenerator.theMixedTeamHeader.isPenalized = theRobotInfo.penalty != PENALTY_NONE;
  outputGenerator.theBHumanStandardMessage.isPenalized = theRobotInfo.penalty != PENALTY_NONE;

  outputGenerator.theBSPLStandardMessage.fallen =
    !outputGenerator.theBHumanStandardMessage.hasGroundContact || !outputGenerator.theBHumanStandardMessage.isUpright;

  SEND_PARTICLE(BNTP);

  SEND_PARTICLE(BallModel);

  if(sendMirroredRobotPose)
  {
    RobotPose theMirroredRobotPose = theRobotPose;
    theMirroredRobotPose.translation *= -1.f;
    theMirroredRobotPose.rotation = Angle::normalize(theMirroredRobotPose.rotation + pi);
    SEND_PARTICLE(MirroredRobotPose);
  }
  else
    SEND_PARTICLE(RobotPose);

  SEND_PARTICLE(SideConfidence);
  SEND_PARTICLE(BehaviorStatus);
  SEND_PARTICLE(TeamBehaviorStatus);

  SEND_PARTICLE(Whistle);

  //Send this last of important data, because they are the biggest
  SEND_PARTICLE(ObstacleModel);
  SEND_PARTICLE(FieldCoverage);

  //Send this last, because it is unimportant for robots, (so it is ok, if it gets dropped)
  SEND_PARTICLE(RobotHealth);
  SEND_PARTICLE(FieldFeatureOverview);
  SEND_PARTICLE(TeamTalk);

  outputGenerator.theBSPLStandardMessage.numOfDataBytes =
    static_cast<uint16_t>(outputGenerator.theBHumanStandardMessage.sizeOfBHumanMessage()
                          + outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage());
}

void TeamMessageHandler::writeMessage(BHumanMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage* const m) const
{
  ASSERT(outputGenerator.sendThisFrame);

  outputGenerator.theMixedTeamHeader.write(reinterpret_cast<void*>(m->data));
  int offset = MixedTeamHeader::sizeOfMixedTeamHeader();
  outputGenerator.theBHumanStandardMessage.write(reinterpret_cast<void*>(m->data + offset));
  offset += outputGenerator.theBHumanStandardMessage.sizeOfBHumanMessage();

  const int restBytes = SPL_STANDARD_MESSAGE_DATA_SIZE - offset;
  ASSERT(restBytes > 10);

  int sizeOfArbitraryMessage;
  if((sizeOfArbitraryMessage = outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage()) > restBytes)
  {
    OUTPUT_ERROR("outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage() > restBytes "
                 "-- with size of " << sizeOfArbitraryMessage << " and restBytes " << int(restBytes));

    do
      outputGenerator.theBHumanArbitraryMessage.queue.removeLastMessage();
    while((sizeOfArbitraryMessage = outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage()) > restBytes
          && !outputGenerator.theBHumanArbitraryMessage.queue.isEmpty());
  }

  ASSERT(sizeOfArbitraryMessage < restBytes);

  outputGenerator.theBHumanArbitraryMessage.write(reinterpret_cast<void*>(m->data + offset));

  outputGenerator.theBSPLStandardMessage.numOfDataBytes = static_cast<uint16_t>(offset + sizeOfArbitraryMessage);
  outputGenerator.theBSPLStandardMessage.write(reinterpret_cast<void*>(&m->header[0]));

  outputGenerator.sentMessages++;
  if(theFrameInfo.getTimeSince(timeLastSent) >= 2 * sendInterval)
    timeLastSent = theFrameInfo.time;
  else
    timeLastSent += sendInterval;
}

void TeamMessageHandler::update(TeamData& teamData)
{
  teamData.generate = [this, &teamData](const RoboCup::SPLStandardMessage* const m)
  {
    if(readSPLStandardMessage(m))
      return parseMessageIntoBMate(getBMate(teamData));

    if(receivedMessageContainer.lastErrorCode == ReceivedBHumanMessage::myOwnMessage
#ifndef NDEBUG
       || receivedMessageContainer.lastErrorCode == ReceivedBHumanMessage::magicNumberDidNotMatch
#endif
      ) return;

    //the message had an parsing error
    if(theFrameInfo.getTimeSince(timeWhenLastMimimi) > minTimeBetween2RejectSounds && SystemCall::playSound("intruderAlert.wav"))
      timeWhenLastMimimi = theFrameInfo.time;

    ANNOTATION("intruder-alert", "error code: " << receivedMessageContainer.lastErrorCode);
  };

  maintainBMateList(teamData);
}

void TeamMessageHandler::maintainBMateList(TeamData& teamData) const
{
  //@author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
  {
    // Iterate over deprecated list of teammate information and update some convenience information
    // (new information has already been coming via handleMessages)
    for(auto& teammate : teamData.teammates)
    {
      Teammate::Status newStatus = Teammate::PLAYING;
      if(teammate.isPenalized || theOwnTeamInfo.players[teammate.number - 1].penalty != PENALTY_NONE)
        newStatus = Teammate::PENALIZED;
      else if(!teammate.isUpright || !teammate.hasGroundContact)
        newStatus = Teammate::FALLEN;

      if(newStatus != teammate.status)
      {
        teammate.status = newStatus;
        teammate.timeWhenStatusChanged = theFrameInfo.time;
      }

      teammate.isGoalkeeper = teammate.number == 1;
    }

    // Remove elements that are too old:
    auto teammate = teamData.teammates.begin();
    while(teammate != teamData.teammates.end())
    {
      if(theFrameInfo.getTimeSince(teammate->timeWhenLastPacketReceived) > networkTimeout)
        teammate = teamData.teammates.erase(teammate);
      else
        ++teammate;
    }

    // Other stuff
    teamData.numberOfActiveTeammates = 0;
    teammate = teamData.teammates.begin();
    while(teammate != teamData.teammates.end())
    {
      if(teammate->status != Teammate::PENALIZED)
        teamData.numberOfActiveTeammates++;
      ++teammate;
    }
  }
}

#define PARSING_ERROR(outputText) { OUTPUT_ERROR(outputText); receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::parsingError;  return false; }
bool TeamMessageHandler::readSPLStandardMessage(const RoboCup::SPLStandardMessage* const m)
{
  if(!receivedMessageContainer.theBSPLStandardMessage.read(&m->header[0]))
    PARSING_ERROR("BSPL" " message part reading failed");

#ifndef SELF_TEST
  if(receivedMessageContainer.theBSPLStandardMessage.playerNum == theRobotInfo.number)
    return (receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::myOwnMessage) && false;
#endif // !SELF_TEST

  if(receivedMessageContainer.theBSPLStandardMessage.playerNum < Settings::lowestValidPlayerNumber ||
     receivedMessageContainer.theBSPLStandardMessage.playerNum > Settings::highestValidPlayerNumber)
    PARSING_ERROR("Invalid robot number received");

  if(receivedMessageContainer.theBSPLStandardMessage.teamNum != static_cast<uint8_t>(Global::getSettings().teamNumber))
    PARSING_ERROR("Invalid team number received");

  if(!receivedMessageContainer.theMixedTeamHeader.read(m->data))
    PARSING_ERROR(MIXED_TEAM_HEADER_STRUCT_HEADER " message part failed");

  // B&B: B-Human provides the robots with numbers 1, 2 and 3.
  if(theRawGameInfo.competitionType != COMPETITION_TYPE_7V7
     || ((receivedMessageContainer.theBSPLStandardMessage.playerNum >= 1 && receivedMessageContainer.theBSPLStandardMessage.playerNum <= 3)
         == (theRobotInfo.number >= 1 && theRobotInfo.number <= 3)))
  {
    size_t offset = MixedTeamHeader::sizeOfMixedTeamHeader();
    if(!receivedMessageContainer.theBHumanStandardMessage.read(m->data + offset))
      PARSING_ERROR(BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER " message part reading failed");

    if(receivedMessageContainer.theBHumanStandardMessage.magicNumber != Global::getSettings().magicNumber)
      return (receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::magicNumberDidNotMatch) && false;

    offset += receivedMessageContainer.theBHumanStandardMessage.sizeOfBHumanMessage();
    if(!receivedMessageContainer.theBHumanArbitraryMessage.read(m->data + offset))
      PARSING_ERROR(BHUMAN_ARBITRARY_MESSAGE_STRUCT_HEADER " message part reading failed");

    receivedMessageContainer.hasBHumanParts = true;
  }
  else
    receivedMessageContainer.hasBHumanParts = false;

  return true;
}

Teammate& TeamMessageHandler::getBMate(TeamData& teamData) const
{
  teamData.receivedMessages++;

  for(auto& teammate : teamData.teammates)
    if(teammate.number == receivedMessageContainer.theBSPLStandardMessage.playerNum)
      return teammate;

  teamData.teammates.emplace_back();
  return teamData.teammates.back();
}

#define RECEIVE_PARTICLE(particle) currentTeammate.the##particle << receivedMessageContainer
void TeamMessageHandler::parseMessageIntoBMate(Teammate& currentTeammate)
{
#ifndef TARGET_ROBOT
  // B&B:
  // In the first frame(s) in which messages are received in the simulator, theRawGameInfo.competitionType is not MIXEDTEAM.
  // As soon as this has changed, the already created teammates must be reset to default values.
  if(currentTeammate.mateType == Teammate::BHumanRobot && !receivedMessageContainer.hasBHumanParts)
    currentTeammate = Teammate();
  // In real games this should never happen because whether this is a mixed team game is checked using the teamNumber which is already known at startup.
#endif

  currentTeammate.number = receivedMessageContainer.theBSPLStandardMessage.playerNum;
  currentTeammate.mateType = receivedMessageContainer.hasBHumanParts ? Teammate::BHumanRobot : Teammate::otherTeamRobot;

  if(receivedMessageContainer.hasBHumanParts)
  {
    theBNTP << receivedMessageContainer;

    receivedMessageContainer.bSMB = theBNTP[currentTeammate.number];
    currentTeammate.bSMB = theBNTP[currentTeammate.number];
    currentTeammate.timeWhenLastPacketSent = receivedMessageContainer.toLocalTimestamp(receivedMessageContainer.theBHumanStandardMessage.timestamp);
    currentTeammate.timeWhenLastPacketReceived = Time::getCurrentSystemTime();

    currentTeammate.isUpright = receivedMessageContainer.theBHumanStandardMessage.isUpright;
    currentTeammate.timeOfLastGroundContact = receivedMessageContainer.toLocalTimestamp(receivedMessageContainer.theBHumanStandardMessage.timeOfLastGroundContact);
    currentTeammate.hasGroundContact = receivedMessageContainer.theBHumanStandardMessage.hasGroundContact;

    currentTeammate.isPenalized = receivedMessageContainer.theBHumanStandardMessage.isPenalized;
  }
  else
  {
    receivedMessageContainer.bSMB = nullptr;
    currentTeammate.bSMB = nullptr;
    currentTeammate.timeWhenLastPacketReceived = Time::getCurrentSystemTime();
    currentTeammate.timeWhenLastPacketSent = currentTeammate.timeWhenLastPacketReceived - 200;

    currentTeammate.isUpright = !receivedMessageContainer.theBSPLStandardMessage.fallen;
    currentTeammate.hasGroundContact = !receivedMessageContainer.theBSPLStandardMessage.fallen;
    if(currentTeammate.hasGroundContact)
      currentTeammate.timeOfLastGroundContact = currentTeammate.timeWhenLastPacketReceived - 200;

    currentTeammate.isPenalized = receivedMessageContainer.theMixedTeamHeader.isPenalized;
  }

  RECEIVE_PARTICLE(SideConfidence);
  RECEIVE_PARTICLE(RobotPose);
  RECEIVE_PARTICLE(BallModel);
  RECEIVE_PARTICLE(ObstacleModel);
  RECEIVE_PARTICLE(BehaviorStatus);
  RECEIVE_PARTICLE(Whistle);
  RECEIVE_PARTICLE(TeamBehaviorStatus);
  RECEIVE_PARTICLE(FieldCoverage);
  RECEIVE_PARTICLE(RobotHealth);
  RECEIVE_PARTICLE(TeamTalk);

  if(receivedMessageContainer.hasBHumanParts)
    receivedMessageContainer.theBHumanArbitraryMessage.queue.handleAllMessages(currentTeammate);
  }

    void TeamMessageHandler::MandarMensaje()
  {

    sendInterval = 5000;

  }

    void TeamMessageHandler::NoMandarMensaje()
  {

    sendInterval = 100000;

  }

    void TeamMessageHandler::MensajeSporadico()
  {
    sendInterval = 1000;
  }