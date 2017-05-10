#include "../../BehaviorControl2015/LibraryBase.h"

namespace Behavior2015
{
#include "LibStabber.h"
#include "LibBall.h"
#include "LibRobot.h"
#include "LibGoals.h"
#include "LibObstacle.h"
#include "LibTeammate.h"

  LibStabber::LibStabber()
  {
  }

  void LibStabber::preProcess()
  {

  }

  void LibStabber::postProcess()
  {
  }

  Pose2f LibStabber::StabberPositionWhenBallNearOurGoal()
  {
    Pose2f targetReturn;
    Vector2f target;
    ball.getSuitablePosition();
    target.x() = theFieldDimensions.xPosHalfWayLine + 600;
    target.y() = ball.suitable.y() - 1000;
    if (target.y() > 2000)
      target.y() = 2000;
    if (target.y() < -2000)
      target.y() = -2000;
    targetReturn.translation = Geometry::fieldCoord2Relative(
        Pose2f(robot.rotation, robot.x, robot.y), target);
    targetReturn.rotation = pi * 2 / 3 - robot.rotation;
    return targetReturn;
  }

  Pose2f LibStabber::StabberPositionWhenBallFarFromGoal()
  {
    Pose2f targetReturn;
    Vector2f target;
    ball.getSuitablePosition();
    target.x() = ball.suitable.x() + 3600;
    if (target.x() > 3600)
      target.x() = 3600;
    if (robot.y < 0)
    {
      target.y() = (target.x() - 600) * (2000 - 1100) / 3000 - 2000;
      targetReturn.translation = Geometry::fieldCoord2Relative(
          Pose2f(robot.rotation, robot.x, robot.y), target);
      targetReturn.rotation = pi * 0.667f - robot.rotation;
    }
    else
    {
      target.y() = 2000 - (target.x() - 600) * (2000 - 1100) / 3000;
      targetReturn.translation = Geometry::fieldCoord2Relative(
          Pose2f(robot.rotation, robot.x, robot.y), target);
      targetReturn.rotation = - pi * 0.667f - robot.rotation;
    }
    return targetReturn;
  }

  BallModel LibStabber::getBallPosition()
  {
    BallModel ballModel;
    for(const Teammate &teammate : theTeammateData.teammates)
    {
      if(teammate.role.role == Role::stabber)
        ballModel = teammate.ball;
    }
    return ballModel;
  }

  bool LibStabber::isTooCloseToStriker()
  {
    if(!teammate.striker.isOnline)
      return false;
    Pose2f strikerPose = teammate.striker.pose;
    float distance = (strikerPose.translation - robot.pose.translation).norm();
//    if(distance < 400 && obstacle.distance < 300)//为什么还要考虑到障碍的距离,在仿真中会出现bug,stabber一直在avoid striker
    if(distance < 400)
      return true;
    else
      return false;
  }

  Pose2f LibStabber::StabberPositionWhenBallNearOppoGoal()
  {
    Pose2f targetReturn;
    Vector2f target;
    ball.getSuitablePosition();
    target.x() = ball.suitable.x() - 500;
    target.y() = ball.suitable.y() + (ball.suitable.y() < 0 ? -500 : 500);
    targetReturn.translation = Geometry::fieldCoord2Relative(
       robot.pose , target);
    targetReturn.rotation = (goals.angleToGoal + ball.angleRad) * 0.5f;
    return targetReturn;
  }

  Pose2f LibStabber::StabberPosition()
  {
    ball.getSuitablePosition();
    if (ball.suitable.x() < -3000)
      return StabberPositionWhenBallNearOurGoal();
    else if (ball.suitable.x() < -2000)
    {
      Pose2f nearOurGoal = StabberPositionWhenBallNearOurGoal();
      Pose2f farFromGoal = StabberPositionWhenBallFarFromGoal();
      Pose2f targetReturn;
      float a = (ball.suitable.x() + 3000) / 1000;
      targetReturn.translation.x() = (1 - a) * nearOurGoal.translation.x()
          + a * farFromGoal.translation.x();
      targetReturn.translation.y() = (1 - a) * nearOurGoal.translation.y()
          + a * farFromGoal.translation.y();
      targetReturn.rotation = (1 - a) * nearOurGoal.rotation
          + a * farFromGoal.rotation;
      return targetReturn;
    }
    else if (ball.suitable.x() < 3000)
    {
      return StabberPositionWhenBallFarFromGoal();
    }
    else
      return StabberPositionWhenBallNearOppoGoal();
  }
  
  /*
  *@ point field point
  *@ obstacles field point
  *@ return 1--occupied
  *         0--not occupied
  */
  bool LibStabber::isPointOccupied(Vector2f point,std::vector<Vector2f> obstacles)
  {
  	Vector2f dis;
	float distance;
	std::vector<Vector2f>::iterator obs;
	for(obs=obstacles.begin();obs!=obstacles.end();obs++)
	{
		dis=(*obs)-point;
		distance=dis.norm();
		if(distance<250.f)
			return true;
	}
	return false;
  }
}
