#include "../LibraryBase.h"

namespace Behavior2015
{
#include "LibBehavior.h"
#include "LibBall.h"
#include "LibCommon.h"
#include "LibGoals.h"
#include "LibRobot.h"
#include "LibTeammate.h"

  LibBehavior::LibBehavior()
  {
  }

  void LibBehavior::preProcess()
  {
    init();
    roleRequestWhenReadyState();
  }
  void LibBehavior::postProcess()
  {
    updateOwnRoleBehavior();
    updateSPLStandardBehaviorStatus();
  }

  void LibBehavior::init()
  {
    getRoleBehavior();
    resetOwnRoleBehavior();
  }

  void LibBehavior::getRoleBehavior()
  {
    keeper = theTeammateData.roleBehavior.keeperBehavior;
    striker = theTeammateData.roleBehavior.strikerBehavior;
    supporter = theTeammateData.roleBehavior.supporterBehavior;
    defender = theTeammateData.roleBehavior.defenderBehavior;
    stabber = theTeammateData.roleBehavior.stabberBehavior;
  }

  void LibBehavior::resetOwnRoleBehavior()
  {
    switch(theRole.role)
    {
    case Role::keeper:
      keeperOutput = KeeperBehavior();
      othersControlBall = striker.isControlBall || supporter.isControlBall || defender.isControlBall || stabber.isControlBall;
      break;
    case Role::striker:
      strikerOutput = StrikerBehavior();
      othersControlBall = keeper.isControlBall || supporter.isControlBall || defender.isControlBall || stabber.isControlBall;
      break;
    case Role::supporter:
      supporterOutput = SupporterBehavior();
      supporterOutput.kickOffDirection = theRoleBehavior.supporterBehavior.kickOffDirection;
      othersControlBall = keeper.isControlBall || striker.isControlBall || defender.isControlBall || stabber.isControlBall;
      break;
    case Role::defender:
      defenderOutput = DefenderBehavior();
      othersControlBall = keeper.isControlBall || striker.isControlBall || supporter.isControlBall || stabber.isControlBall;
      break;
    case Role::stabber:
      stabberOutput = StabberBehavior();
      othersControlBall = keeper.isControlBall || striker.isControlBall || supporter.isControlBall || defender.isControlBall;
      break;
    default:
      break;
    }
  }

  void LibBehavior::updateOwnRoleBehavior()
  {
    switch(theRole.role)
    {
    case Role::keeper:
      keeperOutput.valid = true;
      theRoleBehavior.keeperBehavior = keeperOutput;
      break;
    case Role::striker:
      strikerOutput.valid = true;
      theRoleBehavior.strikerBehavior = strikerOutput;
      break;
    case Role::supporter:
      supporterOutput.valid = true;
      theRoleBehavior.supporterBehavior = supporterOutput;
      break;
    case Role::defender:
      defenderOutput.valid = true;
      theRoleBehavior.defenderBehavior = defenderOutput;
      break;
    case Role::stabber:
      stabberOutput.valid = true;
      theRoleBehavior.stabberBehavior = stabberOutput;
      break;
    default:
      break;
    }
  }

  void LibBehavior::updateSPLStandardBehaviorStatus()
  {

  }

  void LibBehavior::roleRequestWhenReadyState()
  {
    if(common.gameState == STATE_PLAYING)
    {
      readyAfterPlaying = false;
      return;
    }
    if(common.gameState == STATE_READY && common.lastGameState == STATE_PLAYING)
    {
      strikerOutput.readyAfterPlaying = true;
      readyAfterPlaying = true;
    }
    if(readyAfterPlaying)
    {
      if(theGameInfo.kickOffTeam != theOwnTeamInfo.teamNumber)
        roleRequestWhenReadyState(Role::striker);
      else
        roleRequestWhenReadyState(Role::supporter);
    }
  }

  void LibBehavior::roleRequestWhenReadyState(Role::RoleType role)
  {
    if(role == Role::striker)
    {
      if(theRole.role == Role::striker)
        return;
      float strikerPositionDistance;
      if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
        strikerPositionDistance = (robot.pose.translation - robot.strikerReadyPoseWhenKickOff.translation).norm();
      else
        strikerPositionDistance = (robot.pose.translation - robot.strikerReadyPose.translation).norm();
      for(const Teammate &teammate : theTeammateData.teammates)
      {
//        if(teammate.role.role != Role::striker)
//          continue;
        float distance;
        if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
          distance = (teammate.pose.translation - robot.strikerReadyPoseWhenKickOff.translation).norm();
        else
          distance = (teammate.pose.translation - robot.strikerReadyPose.translation).norm();
        if(strikerPositionDistance >= distance)
        {
          return;
        }
      }
//      if(theRole.role == Role::supporter)
//        std::cout<<"request----------"<<std::endl;
      if(theRole.role == Role::supporter)
        supporterOutput.requestRoleType = Role::striker;
      else if(theRole.role == Role::defender)
        defenderOutput.requestRoleType = Role::striker;
      else if(theRole.role == Role::stabber)
        stabberOutput.requestRoleType = Role::striker;
    }
    else if(role == Role::supporter)
    {
      if(theRole.role == Role::supporter)
        return;
      float supporterPositionDistance;
      if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
        supporterPositionDistance = (robot.pose.translation - robot.supporterReadyPoseWhenKickOff.translation).norm();
      else
        supporterPositionDistance = (robot.pose.translation - robot.supporterReadyPose.translation).norm();
      for(const Teammate &teammate : theTeammateData.teammates)
      {
        float distance;
        if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
          distance = (teammate.pose.translation - robot.supporterReadyPoseWhenKickOff.translation).norm();
        else
          distance = (teammate.pose.translation - robot.supporterReadyPose.translation).norm();
        if(supporterPositionDistance >= distance)
        {
          return;
        }
      }
      if(theRole.role == Role::striker)
        strikerOutput.requestRoleType = Role::supporter;
      else if(theRole.role == Role::defender)
        defenderOutput.requestRoleType = Role::supporter;
      else if(theRole.role == Role::stabber)
        stabberOutput.requestRoleType = Role::supporter;
    }
  }

  bool LibBehavior::isOtherTeammateControlBall()
  {
    bool result = false;
    switch(theRole.role)
    {
    case Role::keeper:
      result = striker.isControlBall || supporter.isControlBall || defender.isControlBall || stabber.isControlBall;
      break;
    case Role::striker:
      result = keeper.isControlBall || supporter.isControlBall || defender.isControlBall || stabber.isControlBall;
      break;
    case Role::supporter:
      result = keeper.isControlBall || striker.isControlBall || defender.isControlBall || stabber.isControlBall;
      break;
    case Role::defender:
      result = keeper.isControlBall || striker.isControlBall || supporter.isControlBall || stabber.isControlBall;
      break;
    case Role::stabber:
      result = keeper.isControlBall || striker.isControlBall || supporter.isControlBall || defender.isControlBall;
      break;
    default:
      break;
    }
    return result;
  }

  float LibBehavior::timeToReachBall()
  {
    const float walkSpeed = 150.f;
    const float turnSpeed = 20_deg;
    float ballDistance = ball.distance;
    float turnAngle = goals.angleToGoal;
    float timeToReachBall = ballDistance / walkSpeed + turnAngle / turnSpeed;
    return timeToReachBall;
  }
}
