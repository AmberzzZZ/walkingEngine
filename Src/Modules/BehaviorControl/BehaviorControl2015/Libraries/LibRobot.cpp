/**
 * @file LibGoals.cpp
 * @author Hanbin Wang
 * @date Feb 27, 2014
 */
//#include "Representations/Perception/BackgroundPercept.h" // using global variance, ugly but works
#include "../../BehaviorControl2015/LibraryBase.h"

#include <iostream>

namespace Behavior2015
{
#include "../../BehaviorControl2015/Libraries/LibRobot.h"
#include "../../BehaviorControl2015/Libraries/LibArea.h"

  LibRobot::LibRobot()
      : angleToGoal(0.f),
        numKeeper(1),
        numStriker(2),
        numSupporter(3),
        numDefender(4),
        numStabber(5),
        keeperReadyPose(Pose2f(0.f, -4250.f, 0.f)),
        strikerReadyPose(Pose2f(0.f, -1000.f, 0.f)), // 改的话要同时改动RoleProvider的数据
        strikerReadyPoseWhenKickOff(Pose2f(0.f, -750.f, 1500.f)), // 改的话要同时改动RoleProvider的数据
        supporterReadyPose(Pose2f(0.f, -2000.f, 1500.f)),
        supporterReadyPoseWhenKickOff(Pose2f(0.f, -500.f, 0.f)),
        defenderReadyPose(Pose2f(0.f, -3200.f, -1100.f)),
        stabberReadyPose(Pose2f(0.f, -750.f, -1500.f))
  {
  }

  void LibRobot::preProcess()
  {
    if(theRole.currentKeeper.size() == 1)
      numKeeper = theRole.currentKeeper[0];
    if(theRole.currentStriker.size() == 1)
      numStriker = theRole.currentStriker[0];
    if(theRole.currentSupporter.size() == 1)
      numSupporter = theRole.currentSupporter[0];
    if(theRole.currentDefender.size() == 1)
      numDefender = theRole.currentDefender[0];
    if(theRole.currentStabber.size() == 1)
      numStabber = theRole.currentStabber[0];

    kickOffTargetPoint.x() = 800.f;
    kickOffTargetPoint.y() = 0.f;

    pose = theRobotPose;
    angleToGoal = (theRobotPose.inverse()
        * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
//    std::cout<<"------------------------------------------"<<std::endl;
//    std::cout<<"angleToGoal:"<<angleToGoal * 180.f /sh tj 3.1415926f<<std::endl;
//    std::cout<<"------------------------------------------"<<std::endl;
    odometry = theOdometer.odometryOffset;
  }

  void LibRobot::postProcess()
  {
  }

  bool LibRobot::isFallen()
  {
    return (theFallDownState.state != FallDownState::upright
        && theFallDownState.state != FallDownState::undefined);
  }

  bool LibRobot::isNearOppGoalLeft()
  {
    return area.nearOppGoalLeft(x, y);
  }
  bool LibRobot::isNearOppGoalRight()
  {
    return area.nearOppGoalRight(x, y);
  }
  bool LibRobot::isNearOppGoalMiddle()
  {
    return area.nearOppGoalMiddle(x, y);
  }
  void LibRobot::setLearning(bool learn)
  {
//    ready = learn;
  }
  int LibRobot::sumOpponentNearLeft()
  {
	  int sumoppponent = 0;
	  for(int i = 0;i < 3;i++)
		  sumoppponent = sumoppponent + opponentCount[i];
	  return sumoppponent;
  }

  int LibRobot::sumOpponentNearRight()
  {
	  int sumoppponent = 0;
	  for(int i = 3;i < 6;i++)
		  sumoppponent = sumoppponent + opponentCount[i];
	  return sumoppponent;
  }

  int LibRobot::sumOpponentFarLeft()
  {
	  int sumoppponent = 0;
	  for(int i = 0;i < 3;i++)
		  sumoppponent = sumoppponent + opponentFarCount[i];
	  return sumoppponent;
  }

  int LibRobot::sumOpponentFarRight()
  {
	  int sumoppponent = 0;
	  for(int i = 3;i < 6;i++)
		  sumoppponent = sumoppponent + opponentFarCount[i];
	  return sumoppponent;
  }
}
