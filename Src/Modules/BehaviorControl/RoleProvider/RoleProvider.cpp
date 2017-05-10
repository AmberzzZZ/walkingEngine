/*
 *@file RoleProvider.cpp
 * this file implements the module that provides the role of the robot
 */
#include "RoleProvider.h"
#include <iostream>
#include <vector>
#define DEBUG_NUMBER 3
MAKE_MODULE(RoleProvider, behaviorControl)  //change Behavior Control to behaviorControl @YANG

RoleProvider::RoleProvider(){}

//kind 1-keeper 2-striker 3-supporter 5-breakingsupporter 4-defender
void RoleProvider::update(Role& role)
{
//  lastGameState = gameState;
//  gameState = theGameInfo.state;
//  if(gameState == STATE_SET && lastGameState == STATE_READY)
//  {
//    role.isKickOff = true;
//    role.isStabberReady= false;
//  }

  //当主动要求切换角色后，在一段时间内不允许其他队员再切换成自己的当前角色
  role.isRequestNotAllowed = theFrameInfo.getTimeSince(role.lastRequestTime) > 5000? false:true;

  //更新整个团队中的role的信息
  getRoleDataOnline(role);

  //initial状态下，根据机器人号码初始化
  if(theGameInfo.state == STATE_INITIAL)
  {
    initial(role);
    return;
  }
//  initial(role);
//  if(theGameInfo.state == STATE_SET)
//     getKickOffChoiceByStriker(role);

  //非Set或者Playing状态下不改变角色信息
//  if(theGameInfo.state != STATE_PLAYING && theGameInfo.state != STATE_SET)
//    return;

  //如果收到其他队员的角色切换要求，切换成该队员的先前角色
  changeRoleIfReceiveRequest(role);

  if(role.requestType != Role::none)
  {
    resetRequestType(role);//当场上不存在所请求切换的角色，直接切换，不用发送请求，防止没有回复，发生冲突
  }

//  检查是否发生多人担任同一角色的情况
  checkRoleAssign(role);

//  根据不同角色来确定角色切换的策略
  switch(role.role)
  {
  case Role::keeper:
    break;
  case Role::striker:
    strikerRoleAssignDecision(role);
    break;
  case Role::supporter:
    supporterRoleAssignDecision(role);
    break;
  case Role::defender:
    defenderRoleAssignDecision(role);
    break;
  case Role::stabber:
    stabberRoleAssignDecision(role);
    break;
  default:
    break;
  }

//  stateUpdate(role);
}

void RoleProvider::initial(Role& role)
{
    switch(theRobotInfo.number)
    {
      case 1:
        role.role = Role::keeper;
        role.requestType = Role::none;
        break;
      case 2:
        role.role = Role::striker;
        role.requestType = Role::none;
        break;
      case 3:
        role.role = Role::supporter;
        role.requestType = Role::none;
        break;
      case 4:
        role.role = Role::defender;
        role.requestType = Role::none;
        break;
      case 5:
        role.role = Role::stabber;
        role.requestType = Role::none;
        break;
      default:
        role.role = Role::striker;
        role.requestType = Role::none;
        break;
    }
}

void RoleProvider::getRoleDataOnline(Role &role)
{
  numOnline = theTeammateData.teammates.size();
  role.currentKeeper.clear();
  role.currentStriker.clear();
  role.currentSupporter.clear();
  role.currentDefender.clear();
  role.currentStabber.clear();
  static unsigned int currentKeeperCnt = 100, currentStrikerCnt = 100, currentSupporterCnt = 100, currentDefenderCnt = 100, currentStabberCnt = 100;
  const unsigned int delayCnt = 200;
  bool findKeeper = false, findStriker = false, findSupporter = false, findDefender = false, findStabber = false;

  switch(role.role)
  {
    case Role::keeper:
      role.currentKeeper.push_back(theRobotInfo.number);
      findKeeper = true;
      break;
    case Role::striker:
      role.currentStriker.push_back(theRobotInfo.number);
      findStriker = true;
      break;
    case Role::supporter:
      role.currentSupporter.push_back(theRobotInfo.number);
      findSupporter = true;
      break;
    case Role::defender:
      role.currentDefender.push_back(theRobotInfo.number);
      findDefender = true;
      break;
    case Role::stabber:
      role.currentStabber.push_back(theRobotInfo.number);
      findStabber = true;
      break;
    default:
      break;
  }

  for(const Teammate &teammate : theTeammateData.teammates)
  {
    switch(teammate.role.role)
    {
      case Role::keeper:
        role.currentKeeper.push_back(teammate.number);
        findKeeper = true;
        break;
      case Role::striker:
        role.currentStriker.push_back(teammate.number);
        findStriker = true;
        break;
      case Role::supporter:
        role.currentSupporter.push_back(teammate.number);
        findSupporter = true;
        break;
      case Role::defender:
        role.currentDefender.push_back(teammate.number);
        findDefender = true;
        break;
      case Role::stabber:
        role.currentStabber.push_back(teammate.number);
        findStabber = true;
        break;
      default:
        break;
    }

  }

  if(findKeeper)
    currentKeeperCnt = currentKeeperCnt < delayCnt? currentKeeperCnt + 5:delayCnt;
  else
    currentKeeperCnt = currentKeeperCnt > 0? currentKeeperCnt - 1:0;
  if(currentKeeperCnt >= delayCnt && role.isKeeperOnline == false)
    role.isKeeperOnline = true;
  else if(currentKeeperCnt <= 0 && role.isKeeperOnline == true)
    role.isKeeperOnline = false;

  if(findStriker)
    currentStrikerCnt = currentStrikerCnt < delayCnt? currentStrikerCnt + 5:delayCnt;
  else
    currentStrikerCnt = currentStrikerCnt > 0? currentStrikerCnt - 1:0;
  if(currentStrikerCnt >= delayCnt && role.isStrikerOnline == false)
    role.isStrikerOnline = true;
  else if(currentStrikerCnt <= 0 && role.isStrikerOnline == true)
    role.isStrikerOnline = false;

  if(findSupporter)
    currentSupporterCnt = currentSupporterCnt < delayCnt? currentSupporterCnt + 5:delayCnt;
  else
    currentSupporterCnt = currentSupporterCnt > 0? currentSupporterCnt - 1:0;
  if(currentSupporterCnt >= delayCnt && role.isSupporterOnline == false)
    role.isSupporterOnline = true;
  else if(currentSupporterCnt <= 0 && role.isSupporterOnline == true)
    role.isSupporterOnline = false;

  if(findDefender)
    currentDefenderCnt = currentDefenderCnt < delayCnt? currentDefenderCnt + 5:delayCnt;
  else
    currentDefenderCnt = currentDefenderCnt > 0? currentDefenderCnt - 1:0;
  if(currentDefenderCnt >= delayCnt && role.isDefenderOnline == false)
    role.isDefenderOnline = true;
  else if(currentDefenderCnt <= 0 && role.isDefenderOnline == true)
    role.isDefenderOnline = false;

  if(findStabber)
    currentStabberCnt = currentStabberCnt < delayCnt? currentStabberCnt + 5:delayCnt;
  else
    currentStabberCnt = currentStabberCnt > 0? currentStabberCnt - 1:0;
  if(currentStabberCnt >= delayCnt && role.isStabberOnline == false)
    role.isStabberOnline = true;
  else if(currentStabberCnt <= 0 && role.isStabberOnline == true)
    role.isStabberOnline = false;
}

void RoleProvider::resetRequestType(Role &role)
{
  for(const Teammate& teammate : theTeammateData.teammates)
  {
    if(teammate.role.role == role.role)
      return;
  }
  role.requestType = Role::none;
}

void RoleProvider::checkRoleAssign(Role &role)
{
  if(role.currentStriker.size() > 1 || role.currentSupporter.size() > 1 || role.currentDefender.size() > 1 ||role.currentStabber.size() > 1)
  {
    if(role.requestType != Role::none)//如果有人请求切换角色，那么表面这次的重复是正常的。
      return;
    for(const Teammate &teammate : theTeammateData.teammates)
    {
      if(teammate.role.requestType != Role::none)
        return;
    }

    std::cout<<"warning(RoleProvider): checkRoleAssign! ("<<theRobotInfo.number<<")"<<std::endl;
    if(role.role == Role::striker && role.currentStriker.size() > 1)
    {
      for(const Teammate &teammate : theTeammateData.teammates)
      {
        if(teammate.role.role != Role::striker)
          continue;
        else if(strikerDecision(role,teammate.number))
          continue;
        else
        {
          if(!role.currentStabber.size())
          {
            role.previewR = role.role;
            role.role = Role::stabber;
          }
          else if(!role.currentSupporter.size())
          {
            role.previewR = role.role;
            role.role = Role::supporter;
          }
          else if(!role.currentDefender.size())
          {
            role.previewR = role.role;
            role.role = Role::defender;
          }
        }
      }
    }
    else if(role.role == Role::supporter && role.currentSupporter.size() > 1)
    {
      for(const Teammate &teammate : theTeammateData.teammates)
      {
        if(teammate.role.role != Role::supporter)
          continue;
        else if(supporterDecision(role,teammate.number))
          continue;
        else
        {
          if(!role.currentStriker.size())
          {
            role.previewR = role.role;
            role.role = Role::striker;
          }
          else if(!role.currentStabber.size())
          {
            role.previewR = role.role;
            role.role = Role::stabber;
          }
          else if(!role.currentDefender.size())
          {
            role.previewR = role.role;
            role.role = Role::defender;
          }
        }
      }
    }
    else if(role.role == Role::defender && role.currentDefender.size() > 1)
    {
      for(const Teammate &teammate : theTeammateData.teammates)
      {
        if(teammate.role.role != Role::defender)
          continue;
        else if(defenderDecision(role,teammate.number))
          continue;
        else
        {
          if(!role.currentStriker.size())
          {
            role.previewR = role.role;
            role.role = Role::striker;
          }
          else if(!role.currentStabber.size())
          {
            role.previewR = role.role;
            role.role = Role::stabber;
          }
          else if(!role.currentSupporter.size())
          {
            role.previewR = role.role;
            role.role = Role::supporter;
          }
        }
      }
    }
    else if(role.role == Role::stabber && role.currentStabber.size() > 1)
    {
      for(const Teammate &teammate : theTeammateData.teammates)
      {
        if(teammate.role.role != Role::stabber)
          continue;
        else if(stabberDecision(role,teammate.number))
          continue;
        else
        {
          if(!role.currentStriker.size())
          {
            role.previewR = role.role;
            role.role = Role::striker;
          }
          else if(!role.currentSupporter.size())
          {
            role.previewR = role.role;
            role.role = Role::supporter;
          }
          else if(!role.currentDefender.size())
          {
            role.previewR = role.role;
            role.role = Role::defender;
          }
        }
      }
    }
  }
  else
    return;
}

//void RoleProvider::stateUpdate(Role &role)
//{
//  if(theRobotInfo.penalty != PENALTY_NONE)
//  {
//    role.isKickBall = role.isControlBall = role.isDuel = false;
//    return;
//  }
//  static unsigned int timeWhenLastKickBall = 0;

////  std::cout<<"timeWhenLastKickBall-> "<<timeWhenLastKickBall<<std::endl;

//  if(role.isKickBall == true && theFrameInfo.getTimeSince(timeWhenLastKickBall) < 4000)
//  {
////    std::cout<<"KickBall"<<std::endl;
//    role.isKickBall = true;
//  }
//  else if(theMotionRequest.motion == MotionRequest::kick ||
//      (theMotionRequest.motion == MotionRequest::specialAction &&
//       (theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::sideKickHardLeft ||
//        theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::sideKickHardRight ||
//        theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::kickBackHard)))
//  {
//    role.isKickBall = true;
//    timeWhenLastKickBall = theFrameInfo.time;
//  }
//  else
//  {
//    role.isKickBall = false;
//  }

//  static unsigned int timeWhenLastDuelTrue = 0;
//  static float obstacleDistance = 3000;
//  if(role.isDuel == true && theFrameInfo.getTimeSince(timeWhenLastDuelTrue) < 2000)
//  {
////    std::cout<<"Duel"<<std::endl;
//    role.isDuel = true;
//  }
//  else
//  {
//    for (const Obstacle & obstacle : theObstacleModel.obstacles)
//    {
//      float d = 3000;
//      if (obstacle.type != Obstacle::opponent || obstacle.center.x() < 0)
//        continue;
//      d = obstacle.center.norm();
//      if (obstacleDistance < d)
//      {
//        obstacleDistance = d;
//      }
//    }
//    if(obstacleDistance < 1000)
//    {
//      role.isDuel = true;
//      timeWhenLastDuelTrue = theFrameInfo.time;
//    }
//    else
//    {
//      role.isDuel = false;
//    }
//  }

//  static unsigned int timeWhenLastControlBall = 0;
//  role.isControlBall = false;
//  role.isOtherTeammatesControlBall = false;
//  for(const Teammate &teammate : theTeammateData.teammates)
//  {
//    if(teammate.role.isControlBall == true)
//    {
//      role.isOtherTeammatesControlBall = true;
////      std::cout<<"role--> "<<role.role<<"   isOtherTeammatesControlBall"<<std::endl;
//    }
//  }
//  if(role.isOtherTeammatesControlBall == false)
//  {
//    unsigned int timeSinceLastControlBall = theFrameInfo.getTimeSince(timeWhenLastControlBall);
//    if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 5000 && theBallModel.estimate.position.norm() < 1500 && theBallModel.estimate.velocity.norm() < 100 && role.isDuel == false)
//    {
//      role.isControlBall = true;
//      timeWhenLastControlBall = theFrameInfo.time;
//    }
//    else if(timeSinceLastControlBall < 1000)
//    {
//      role.isControlBall = true;
//    }
//  }

//  if(role.isKickOff)
//    isKickOffByStriker(role);

//  if(role.role == Role::stabber && !role.isStabberReady)
//    isStabberReady(role);
//}

void RoleProvider::strikerRoleAssignDecision(Role &role)
{
  //上层请求
  if(theRoleBehavior.strikerBehavior.isRequestNotAllowed == true)
    role.isRequestNotAllowed = true;//通知队友不允许替换自己的角色
  if(theRoleBehavior.strikerBehavior.requestRoleType != Role::none)
  {
    requestRole(role, theRoleBehavior.strikerBehavior.requestRoleType);
    return;
  }

//  if(theGameInfo.state == STATE_SET)
//  {
//    Pose2f pose(0.f, -1000.f, 0.f);
//    if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
//      pose = Pose2f(0.f, -300.f, 0.f);
//    if(!theGroundContactState.contact)
//    {
//      requestRole(role, Role::stabber);
//    }
//    else if(fabs(theRobotPose.rotation) > 60_deg || (theRobotPose.translation - pose.translation).norm() > 1500)
//    {
//      Pose2f stabberPose(0.f, -400.f, -1000.f);
//      for(const Teammate &teammate : theTeammateData.teammates)
//      {
//        if(teammate.role.role == Role::stabber)
//        {
//          stabberPose = teammate.pose;
//        }
//      }
//      if(theRobotPose.translation.norm() > stabberPose.translation.norm())
//      {
//        requestRole(role, Role::stabber);
//      }
//    }
//  }
//  return;
}

void RoleProvider::supporterRoleAssignDecision(Role &role)
{
  //上层请求
  if(theRoleBehavior.supporterBehavior.isRequestNotAllowed == true)
    role.isRequestNotAllowed = true;
  if(theRoleBehavior.supporterBehavior.requestRoleType != Role::none)
  {
//    if(role.role == Role::supporter)
//      std::cout<<"post-------------"<<std::endl;
    requestRole(role, theRoleBehavior.supporterBehavior.requestRoleType);
    return;
  }

  //底层请求
  bool isStrikerPenalised = true;
  bool isStabberPenalised = true;
  bool isDefenderPenalised = true;
  if(role.isStrikerOnline)
  {
    for(const Teammate &teammate:theTeammateData.teammates)
    {
      if(teammate.role.role == Role::striker)
      {
        isStrikerPenalised = teammate.isPenalized;
      }
    }
  }
  if(role.isStabberOnline)
  {
    for(const Teammate &teammate:theTeammateData.teammates)
    {
      if(teammate.role.role == Role::stabber)
      {
        isStabberPenalised = teammate.isPenalized;
      }
    }
  }
  if(role.isDefenderOnline)
  {
    for(const Teammate &teammate:theTeammateData.teammates)
    {
      if(teammate.role.role == Role::defender)
      {
        isDefenderPenalised = teammate.isPenalized;
      }
    }
  }
  if(isStrikerPenalised && isStabberPenalised)//只有striker和stabber都不在线的时候才能切换
    requestRole(role, Role::striker);
//  if(isDefenderPenalised)//当striker在线，但是defender不在线时，切换为defender
//	  requestRole(role, Role::defender);
  return;
}

void RoleProvider::defenderRoleAssignDecision(Role &role)
{
  //上层请求
  if(theRoleBehavior.defenderBehavior.isRequestNotAllowed == true)
    role.isRequestNotAllowed = true;
  if(theRoleBehavior.defenderBehavior.requestRoleType != Role::none)
  {
    requestRole(role, theRoleBehavior.defenderBehavior.requestRoleType);
    return;
  }

  //底层请求
//  return;

  bool isStrikerPenalised = true;
  bool isStabberPenalised = true;
  bool isSupporterPenalised = true;
  if(role.isStrikerOnline)
  {
    for(const Teammate &teammate:theTeammateData.teammates)
    {
      if(teammate.role.role == Role::striker)
      {
        isStrikerPenalised = teammate.isPenalized;
      }
    }
  }
  if(role.isStabberOnline)
  {
    for(const Teammate &teammate:theTeammateData.teammates)
    {
      if(teammate.role.role == Role::stabber)
      {
        isStabberPenalised = teammate.isPenalized;
      }
    }
  }
  if(role.isSupporterOnline)
  {
    for(const Teammate &teammate:theTeammateData.teammates)
    {
      if(teammate.role.role == Role::supporter)
      {
        isSupporterPenalised = teammate.isPenalized;
      }
    }
  }
  if(isStrikerPenalised && isStabberPenalised && isSupporterPenalised)//当场上没有进攻球员的时候是否要让defender也上去进攻？
    requestRole(role, Role::striker);
  return;
}

void RoleProvider::stabberRoleAssignDecision(Role &role)
{
  //上层请求
  if(theRoleBehavior.stabberBehavior.isRequestNotAllowed == true)
    role.isRequestNotAllowed = true;
  if(theRoleBehavior.stabberBehavior.requestRoleType != Role::none)
  {
    requestRole(role, theRoleBehavior.stabberBehavior.requestRoleType);
    return;
  }

  //底层请求
  bool isStrikerPenalised = true;
  bool isDefenderPenalised = true;
  if(role.isStrikerOnline)
  {
    for(const Teammate &teammate:theTeammateData.teammates)
    {
      if(teammate.role.role == Role::striker)
      {
        isStrikerPenalised = teammate.isPenalized;
      }
    }
  }
  if(role.isDefenderOnline)
  {
      for(const Teammate &teammate:theTeammateData.teammates)
      {
        if(teammate.role.role == Role::defender)
        {
          isDefenderPenalised = teammate.isPenalized;
        }
      }
  }
  if(isStrikerPenalised)//当striker被发下之后，stabber切换为striker
    requestRole(role, Role::striker);
  if(isDefenderPenalised)
	  requestRole(role, Role::defender);
  return;
//  if(theBallModel.estimate.position.norm() < 2000 && strikerDecision(role))
//    requestRole(role,Role::striker);
}

bool RoleProvider::requestRole(Role &role,Role::RoleType roleType)
{
  if(roleType == Role::keeper)
  {
    return false;
  }
  else
  {
    for(const Teammate& teammate : theTeammateData.teammates)
    {
      if(teammate.role.role != roleType)
        continue;
      if(teammate.role.isRequestNotAllowed)
        return false;
      if(teammate.role.requestType != Role::none)//当对方也需要切换角色时，不用再发出请求，直接切换
        return false;
    }
//    std::cout<<"requestNum--> "<<theRobotInfo.number<<"  type--->"<<roleType<<std::endl;
//    std::cout<<"success-------------- "<<role.role<<", "<<roleType<<std::endl;
    role.requestType = roleType;
    role.previewR = role.role;
    role.role = roleType;
    role.lastRequestTime = theFrameInfo.time;
    return true;
  }
}

void RoleProvider::changeRoleIfReceiveRequest(Role &role)
{
  if(numOnline == 0)
    return;
  for(const Teammate &teammate : theTeammateData.teammates)
  {
    if(teammate.role.requestType == role.role)
    {
      role.previewR = role.role;
      role.role = teammate.role.previewR;
    }
  }
}

bool RoleProvider::strikerDecision(Role& role, int robotNum)//??
{
  if(!role.isStrikerOnline)
    return true;
//  if(role.role != Role::striker && role.currentStriker.size() > 1)//直接就返回false了，是否错误？
//    return false;
  for(const Teammate& teammate : theTeammateData.teammates)
  {
    if(robotNum == 0 && teammate.role.role != Role::striker)
    {
      continue;
    }
    else if(robotNum != 0 && teammate.number != robotNum)
    {
      continue;
    }
    else
    {
      /**@TODO decide who is the best choice of strikker*/
      float ballDistance = theBallModel.estimate.position.norm();
      if(!robotNum)
        ballDistance = ballDistance - 500.f > 0?ballDistance - 500.f:0.f;
//      if(theBallModel.timeToReachBall > teammate.timeToReachBall)
      if(ballDistance > teammate.ball.estimate.position.norm())
        return false;
//      else if(theBallModel.estimate.position.norm() - 500.f == teammate.ball.estimate.position.norm() && theRobotInfo.number > teammate.number)
//        return false;
      else
        return true;
    }
  }
  return true;

//缺少此文件@YANG "Representations/BehaviorControl/TimeToReachBall.h"
//        if (theTimeToReachBall.totalTime<theTeammateData.timeToReachBall[i])
//            overnum++;
//        else if(theTimeToReachBall.totalTime == theTeammateData.timeToReachBall[i]&&theRobotInfo.number<i)
//            overnum++;

// 13年代码的计算方法@WZF
//        if(theCombinedWorldModel.ballIsValid)
//          ballPosition=Geometry::fieldCoord2Relative(theRobotPose,theCombinedWorldModel.ballState.position);
// //       else if(theCombinedWorldModel.ballIsValidOthers)ballPosition=Geometry::fieldCoord2Relative(theRobotPose,theCombinedWorldModel.ballStateOthers.position);
//     else ballPosition=theBallModel.estimate.position;
//      ballGlobalPosition=Geometry::relative2FieldCoord(theRobotPose,ballPosition);
//      opponentGoalPosition=(Vector2<>(theFieldDimensions.xPosOpponentGoal,0));
//      ballToGoal=Geometry::angleTo(ballPosition,opponentGoalPosition-ballGlobalPosition);
  //      timeToReachBall.totalTime=ballPosition.absFloat()/100 + std::fabs(ballPosition.angle())/.5f + 15*(1-theRobotPose.validity);
}

bool RoleProvider::supporterDecision(Role &role, int robotNum)
{
  return false;
}

bool RoleProvider::defenderDecision(Role &role, int robotNum)
{
  return false;
}

bool RoleProvider::stabberDecision(Role &role, int robotNum)
{
  return false;
}
