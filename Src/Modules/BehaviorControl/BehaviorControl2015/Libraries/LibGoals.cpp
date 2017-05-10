/**
 * @file LibGoals.cpp
 */

#include "../LibraryBase.h"

namespace Behavior2015
{
#include "LibCommon.h"
#include "LibBall.h"
#include "LibGoals.h"
#include "LibObstacle.h"

  LibGoals::LibGoals()
      : hasFreePart(false)
  {
  }

  void LibGoals::preProcess()
  {
    angleToGoal = (theRobotPose.inverse()
        * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
//    centerAngleOfOppFreePart();
  }

  void LibGoals::postProcess()
  {

  }

  float LibGoals::centerAngleOfOppFreePart()  //需要改写 @wzf
  {
    const Vector2f leftGoal(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftGoal);
    const Vector2f centerGoal(theFieldDimensions.xPosOpponentGroundline, 0.f);
    const Vector2f rightGoal(theFieldDimensions.xPosOpponentGroundline, -theFieldDimensions.yPosLeftGoal);
    bool isStuck = false;
//    Pose2f ballPoseInverse = (Pose2f(ballToCenter, ball.global)).inverse();

//    float angleToLeftGoal = (ballPoseInverse * leftGoal).angle();
    float angleToLeftGoal = (leftGoal - ball.global).angle();
//    float angleToRightGoal = (ballPoseInverse * rightGoal).angle();
    float angleToRightGoal = (rightGoal - ball.global).angle();
//    float angleToCenter = (ballPoseInverse * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
    float angleToCenter = (centerGoal - ball.global).angle();
    float angleOfLeftFreePart = Geometry::angleTo(Pose2f(angleToRightGoal,ball.global), leftGoal);
    float angleOfRightFreePart = -Geometry::angleTo(Pose2f(angleToLeftGoal,ball.global), rightGoal);

//    float angleToLeftGoal = Geometry::angleTo(theRobotPose, Vector2f(theFieldDimensions.xPosOpponentGroundline, yPosLeftGoal));
//    float angleToRightGoal = Geometry::angleTo(theRobotPose, Vector2f(theFieldDimensions.xPosOpponentGroundline, -yPosLeftGoal));
//    for(const Obstacle &obstacle : theObstacleModel.obstacles)
    for(const Obstacle &obstacle : theObstacleModel.obstacles)
    {
      if(obstacle.type == Obstacle::goalpost || obstacle.type == Obstacle::unknown)
        continue;
      Vector2f centerOnField = Geometry::relative2FieldCoord(theRobotPose, obstacle.center);

      float angleToObstacle = (centerOnField - ball.global).angle();
      if(angleToObstacle > angleToRightGoal && angleToObstacle < angleToLeftGoal && fabs(centerOnField.y()) < 750)
      {
//        std::cout<<"------"<<std::endl;
        isStuck = true;
        Vector2f leftOnField = Geometry::relative2FieldCoord(theRobotPose, obstacle.left);
//        std::cout<<"leftOnField--> ("<<leftOnField.x()<<", "<<leftOnField.y()<<")\n";
        Vector2f rightOnField = Geometry::relative2FieldCoord(theRobotPose, obstacle.right);
//        std::cout<<"rightOnField--> ("<<rightOnField.x()<<", "<<rightOnField.y()<<")\n";
        float angleToObstacleLeft = (leftOnField - ball.global).angle();
        float angleToObstacleRight = (rightOnField - ball.global).angle();
//        std::cout<<"angleToObstacleLeft--> "<<common.fromRad(angleToObstacleLeft)<<std::endl;
//        std::cout<<"angleToObstacleRight--> "<<common.fromRad(angleToObstacleRight)<<std::endl;
        angleOfLeftFreePart = Geometry::angleTo(Pose2f(angleToObstacleLeft, ball.global), leftGoal);
        angleOfRightFreePart = -Geometry::angleTo(Pose2f(angleToObstacleRight, ball.global), rightGoal);
      }
    }
//    std::cout<<"ball.global--> ("<<ball.global.x()<<", "<<ball.global.y()<<")"<<std::endl;
//    std::cout<<"angleToLeftGoal--> "<<common.fromRad(angleToLeftGoal)<<std::endl;
//    std::cout<<"angleToRightGoal--> "<<common.fromRad(angleToRightGoal)<<std::endl;
//    std::cout<<"left--> "<<common.fromRad(angleOfLeftFreePart)<<" ,right-->"<<common.fromRad(angleOfRightFreePart)<<std::endl;

//    std::cout<<"robot.rotation--> "<<common.fromRad(theRobotPose.rotation)<<std::endl;
    if(angleOfLeftFreePart >= angleOfRightFreePart && angleOfLeftFreePart >= 10_deg)
    {
      hasFreePart = true;
//      float robotToLeftGoal = (theRobotPose.inverse() * leftGoal).angle();
//      float robotToLeftGoal = (leftGoal - theRobotPose.translation).angle();
      float robotToLeftGoal = (leftGoal - ball.global).angle();
//      std::cout<<"robotToLeftGoal--> "<<common.fromRad(robotToLeftGoal)<<std::endl;
//      std::cout<<"leftShoot--> "<<common.fromRad(robotToLeftGoal - 0.5f * angleOfLeftFreePart)<<std::endl;
      return robotToLeftGoal - 0.5f * angleOfLeftFreePart - theRobotPose.rotation;
//      return robotToLeftGoal;
    }
    else if(angleOfLeftFreePart < angleOfRightFreePart && angleOfRightFreePart >= 10_deg)
    {
      hasFreePart = true;
//      float robotToRightGoal = (theRobotPose.inverse() * rightGoal).angle();
      float robotToRightGoal = (rightGoal - ball.global).angle();
//      std::cout<<"robotToRightGoal--> "<<common.fromRad(robotToRightGoal)<<std::endl;
//      std::cout<<"rightShoot--> "<<common.fromRad(robotToRightGoal + 0.5f * angleOfRightFreePart)<<std::endl;
      return robotToRightGoal + 0.5f * angleOfRightFreePart - theRobotPose.rotation;
//      return robotToRightGoal;
    }
    else
    {
      hasFreePart = false;
      return angleToGoal;
    }

  }

  float LibGoals::angleToShoot()
  {
    return centerAngleOfOppFreePart();
  }

  float LibGoals::timeSinceOnePostWasSeen()
  {
    return (float) theFrameInfo.getTimeSince(
		theGoalPercept2017.timeWhenLastSeen);
  }

  float LibGoals::timeSinceTwoPostsWasSeen()
  {
    return (float) theFrameInfo.getTimeSince(
        theGoalPercept2017.timeWhenLastSeen);
  }

}
