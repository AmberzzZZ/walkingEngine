/**
 * @file LibTeammate.cpp
 */

#include "../LibraryBase.h"

namespace Behavior2015
{
#include "LibTeammate.h"
#include "LibRobot.h"
#include "LibBall.h"
#include "LibCommon.h"

  LibTeammate::LibTeammate()
  {
  }

  void LibTeammate::preProcess()
  {
//    isKeeperControlBall = isStrikerControlBall = isSupporterControlBall = isDefenderControlBall = isStabberControlBall = false;
//    keeper.isOnline = striker.isOnline = supporter.isOnline = defender.isOnline = stabber.isOnline = false;
    keeper.isOnline = theRole.isKeeperOnline;
    striker.isOnline = theRole.isStrikerOnline;
    supporter.isOnline = theRole.isSupporterOnline;
    defender.isOnline = theRole.isDefenderOnline;
    stabber.isOnline = theRole.isStabberOnline;
    if(theRole.role == Role::keeper)
      setRoleData(keeper);
    else if(theRole.role == Role::striker)
      setRoleData(striker);
    else if(theRole.role == Role::supporter)
      setRoleData(supporter);
    else if(theRole.role == Role::defender)
      setRoleData(defender);
    else if(theRole.role == Role::stabber)
      setRoleData(stabber);
    for(const Teammate &teammate : theTeammateData.teammates)
    {
      if(teammate.role.role == Role::keeper)
      {
//        isKeeperPenalized = teammate.isPenalized;
//        keeperKick = teammate.keeperKick;
        setRoleData(keeper,teammate);
      }
      else if(teammate.role.role == Role::striker)
      {
//        strikerKick = teammate.strikerKick;
        setRoleData(striker,teammate);
      }
      else if(teammate.role.role == Role::supporter)
      {
//        supporterKick = teammate.supporterKick;
        setRoleData(supporter,teammate);
      }
      else if(teammate.role.role == Role::defender)
      {
//        defenderKick = teammate.defenderKick;
        setRoleData(defender,teammate);
      }
      else if(teammate.role.role == Role::stabber)
      {
//        stabberKick = teammate.stabberKick;
        setRoleData(stabber,teammate);
      }
    }
//    isOtherTeammatesControlBall = theRole.isOtherTeammatesControlBall;
  }

  void LibTeammate::postProcess()
  {
  }

  Pose2f LibTeammate::robotPose(int playerNum)
  {
    Pose2f pose(0.f,5000.f,5000.f);
    for (const Teammate& teammate : theTeammateData.teammates)
    {
      if (teammate.number != playerNum)
      {
        continue;
      }
      else
      {
        pose = teammate.pose;
      }
    }
    return pose;
  }

  Teammate::Status LibTeammate::robotState(int playerNum)
    {
      for (const Teammate& teammate : theTeammateData.teammates)
      {
        if (teammate.number != playerNum)
        {
          continue;
        }
        else
        {
          return teammate.status;
        }
      }
      return Teammate::INACTIVE;
    }

  Vector2f LibTeammate::ballGlobalFromRobot(int playerNum)
  {
    Vector2f ballRelativetoRobot;
    Pose2f robotPose;
    for (const Teammate& teammate : theTeammateData.teammates)
    {
    if (teammate.number != playerNum)
    {
      continue;
    }
    else
    {
      if (teammate.status == Teammate::INACTIVE)
        return Vector2f(10000,10000);
      ballRelativetoRobot = teammate.ball.estimate.position;
      robotPose = teammate.pose;
    }

   return Geometry::relative2FieldCoord(
            Pose2f(robotPose.rotation, robotPose.translation.x(),
                   robotPose.translation.y()),
            Vector2f(ballRelativetoRobot.x(), ballRelativetoRobot.y()));
  }
    return Vector2f(10000,10000);

  }
  
  Vector2f LibTeammate::ballRobotFromRobot(int playerNum)
  {
    Vector2f ballRelativetoRobot;
    Pose2f robotPose;
    for (const Teammate& teammate : theTeammateData.teammates)
    {
    if (teammate.number != playerNum)
    {
      continue;
    }
    else
    {
      if (teammate.status == Teammate::INACTIVE)
        return Vector2f(10000,10000);
      ballRelativetoRobot = teammate.ball.estimate.position;
      robotPose = teammate.pose;
    }

   return ballRelativetoRobot;
  }
    return Vector2f(10000,10000);

  }

  //add 0314 @LDR & YJ
  int LibTeammate::getAreaNum(float robotX,float robotY)
  {
     if(robotX > theFieldDimensions.xPosOpponentGroundline)
      robotX = theFieldDimensions.xPosOpponentGroundline - 1;
     else if(robotX < theFieldDimensions.xPosOwnGroundline)
      robotX = theFieldDimensions.xPosOwnGroundline + 1;
    if(robotY > theFieldDimensions.yPosLeftSideline)
      robotY = theFieldDimensions.yPosLeftSideline - 1;
    else if(robotY < theFieldDimensions.yPosRightSideline)
      robotY = theFieldDimensions.yPosRightSideline + 1;
    int x = -(int(robotX)-4500)/1500;
    int y = -(int(robotY)-3000)/2000;
    return (3*x + y);
  }
  int LibTeammate::getStrikerAreaNum()
  {
    if(theRole.currentStriker.empty())
      return 0;
    int strikerNum = theRole.currentStriker.back(); /**@TODO Dynamic Assignment*/
    if(ball.isTeamPositionValid)
    {
      return getAreaNum(ball.teamPosition.x(),ball.teamPosition.y());
    }
    else
    {
      return getAreaNum(robotPose(strikerNum).translation.x(),robotPose(strikerNum).translation.y());
    }
  }

  int LibTeammate::getSupporterCurrentAreaNum()
  {
    return getAreaNum(robot.x,robot.y);
  }
  int LibTeammate::getSupporterNextAreaNum()
  {
    if(theRole.currentStriker.empty())
    {
      std::cout<<"error(supporter): striker not found"<<std::endl;
      return 10;
    }
    int strikerNum = theRole.currentStriker.back();  /**@TODO Dynamic Assignment*/
    int strikerAreaNum = getStrikerAreaNum();
    int supporterCurrentAreaNum = getSupporterCurrentAreaNum();
    std::vector<int> supportPosList(20,10);
    supportPosList[0] = 4;
    supportPosList[1] = 4;
    supportPosList[2] = 4;
    supportPosList[3] = 7;
    supportPosList[4] = 7;
    supportPosList[5] = 7;
    supportPosList[6] =10;
    supportPosList[7] =10;
    supportPosList[8] =10;
    supportPosList[9] =13;
    supportPosList[10]=13;
    supportPosList[11]=13;
    if(strikerAreaNum == 1 || strikerAreaNum == 4 || strikerAreaNum == 7 || strikerAreaNum == 10)
    {
      if((supporterCurrentAreaNum == 3||supporterCurrentAreaNum == 5)&&strikerAreaNum == 1)
        return supporterCurrentAreaNum;
      else if((supporterCurrentAreaNum == 6||supporterCurrentAreaNum == 8)&&strikerAreaNum == 4)
        return supporterCurrentAreaNum;
      else if((supporterCurrentAreaNum == 9||supporterCurrentAreaNum == 11)&&strikerAreaNum == 7)
        return supporterCurrentAreaNum;
      else if((supporterCurrentAreaNum == 12||supporterCurrentAreaNum == 14)&&strikerAreaNum == 10)
          return supporterCurrentAreaNum;
      else{
        if(robotPose(strikerNum).translation.y() >= 0)
          return supportPosList[strikerAreaNum] + 1;
        else
          return supportPosList[strikerAreaNum] - 1;
          }
    }
    else
      return supportPosList[strikerAreaNum];


  }
   Pose2f LibTeammate::getSupporterPose()
   {
     int supportAreaNum = getSupporterNextAreaNum();
     float yPos = 1000.f + 2000.f*(supportAreaNum%3);
     float xPos = 400.f  + 1500.f*(supportAreaNum/3);
     xPos = -(xPos - 4500.f);
     yPos = -(yPos - 3000.f);

//		 if(ball.isTeamPositionValid)
//		 {
//			 return Pose2D(Geometry::relative2FieldCoord(theRobotPose,ball.teamPosition.x,ball.teamPosition.y).angle(),xPos,yPos);
//		 }
//		 else
//		 {
//			return Pose2D(Geometry::relative2FieldCoord(theRobotPose,ball.x,ball.y).angle(),xPos,yPos);
//		 }
//
     return Pose2f(0.f,xPos,yPos);
   }

   void LibTeammate::setRoleData(RoleData &role, const Teammate &teammate)
   {
     role.isPenalized = teammate.isPenalized;
     role.pose = teammate.pose;
     role.ballDistance = teammate.ball.estimate.position.norm();
   }

   void LibTeammate::setRoleData(RoleData &role)
   {
     if(theRobotInfo.penalty == PENALTY_NONE)
       role.isPenalized = false;
     else
       role.isPenalized = true;
     role.pose = Pose2f(theRobotPose.rotation,theRobotPose.translation.x(),theRobotPose.translation.y());
     role.ballDistance = ball.distance;
   }

//   bool LibTeammate::isStabberReadyForKickOff()
//   {
//     bool isReady = false;
//     for(const Teammate& teammate : theTeammateData.teammates)
//     {
//       if(teammate.role.role == Role::stabber)
//       {
//         isReady = teammate.role.isStabberReady;
//       }
//     }
//     return isReady;
//   }

   //end 0314

}

