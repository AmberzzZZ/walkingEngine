#include "../LibraryBase.h"

namespace Behavior2015
{
#include "LibBall.h"
#include "LibDefender.h"
#include "LibKeeper.h"
#include "LibRobot.h"
#include "LibTeammate.h"

  LibDefender::LibDefender ()
  {
  }

  void
  LibDefender::preProcess ()
  {
    ballPositionField = ball.positionField;//getBallPositionField ();
    ballPositionRobot = ball.positionRobot;//getBallPositionRobot ();
    ballSpeedField = ball.speedField;getBallSpeedField ();
    ballSpeedRobot = ball.speedRelative;//getBallSpeedRobot ();
    ballEndPositionRobot = getBallEndPointRobot ();
    ballEndPositionField = getBallEndPointField ();
    yIfBallPassOwnAxis = getYIfBallPassOwnAxis ();
  }

  void
  LibDefender::postProcess ()
  {
  }

  Vector2f
  LibDefender::defenderPositionWhenKeeperPenalized ()
  {
    Vector2f intersectionOfAngleBisectorAndDefenderLine;
    ball.getSuitablePosition ();
    Vector2f goalLeft = Vector2f (theFieldDimensions.xPosOwnGroundline,
				  theFieldDimensions.yPosLeftGoal);
    Vector2f goalRight = Vector2f (theFieldDimensions.xPosOwnGroundline,
				   theFieldDimensions.yPosRightGoal);
    float goalWidth = theFieldDimensions.yPosLeftGoal
	- theFieldDimensions.yPosRightGoal;
    float distanceBallToGoalLeft = (goalLeft - ball.suitable).norm ();
    float distanceBallToGoalRight = (goalRight - ball.suitable).norm ();
    yOnGoal = -(goalWidth * distanceBallToGoalLeft
	/ (distanceBallToGoalLeft + distanceBallToGoalRight)
	- theFieldDimensions.yPosLeftGoal);
    //one of the point of the standardShootLine obtain from intersection of angle bisector and groundLine
    Vector2f onGoal = Vector2f (theFieldDimensions.xPosOwnGroundline, yOnGoal);
    Vector2f lineDirection = Vector2f (ball.suitable.x () - onGoal.x (),
				       ball.suitable.y () - onGoal.y ());
    Geometry::Line angleBisector = Geometry::Line (onGoal, lineDirection);
    if (Geometry::getIntersectionOfLines (
	angleBisector, defenderLine,
	intersectionOfAngleBisectorAndDefenderLine))
      return intersectionOfAngleBisectorAndDefenderLine;
    else
      return Vector2f (-3500.f, 0.f);
  }

  Vector2f
  LibDefender::defenderPositionCoorperateWithKeeperAndSupporter (bool left)
  {
    Vector2f intersectionOfLineOfGoalAndBallAndDefenderLine;
    Vector2f goalLeft = Vector2f (theFieldDimensions.xPosOwnGroundline,
				  theFieldDimensions.yPosLeftGoal - 350.f);
    Vector2f goalRight = Vector2f (theFieldDimensions.xPosOwnGroundline,
				   theFieldDimensions.yPosRightGoal + 350.f);
    Vector2f Goal;
    //one of the point of the standardShootLine obtain from intersection of angle bisector and groundLine
    if (left)
    {
      Goal = goalLeft;
    }
    else
      Goal = goalRight;
    Vector2f lineDirection = Vector2f (ballPositionField.x () - Goal.x (),
				       ballPositionField.y () - Goal.y ());
    Geometry::Line BallAndGoal = Geometry::Line (ballPositionField,
						 lineDirection);
    if (Geometry::getIntersectionOfLines (
	BallAndGoal, defenderLine,
	intersectionOfLineOfGoalAndBallAndDefenderLine))
      return intersectionOfLineOfGoalAndBallAndDefenderLine;
    else
      return Vector2f (-3500.f, 0.f);
  }

  Vector2f
  LibDefender::defenderPositionWhenKeeperNotPenalized ()
  {
    Vector2f defenderPosition;
//    Vector2f intersectionOfMainAngleBisectorAndDefenderLine = defenderPositionWhenKeeperPenalized();
//    Vector2f intersectionOfLeftLineAndDefenderLine;
//    Vector2f intersectionOfRightLineAndDefenderLine;
    Vector2f leftPoint;
    Vector2f rightPoint;
    Vector2f usingPoint;
    Vector2f goalLeft = Vector2f (theFieldDimensions.xPosOwnGroundline,
				  theFieldDimensions.yPosLeftGoal);
    Vector2f goalRight = Vector2f (theFieldDimensions.xPosOwnGroundline,
				   theFieldDimensions.yPosRightGoal);
    Vector2f leftLineDirection = Vector2f (
	ball.suitable.x () - theFieldDimensions.xPosOwnGroundline,
	ball.suitable.y () - theFieldDimensions.yPosLeftGoal);
    Vector2f rightLineDirection = Vector2f (
	ball.suitable.x () - theFieldDimensions.xPosOwnGroundline,
	ball.suitable.y () - theFieldDimensions.yPosRightGoal);
    Geometry::Line leftLine = Geometry::Line (goalLeft, leftLineDirection);
    Geometry::Line rightLine = Geometry::Line (goalRight, rightLineDirection);
    if (Geometry::getIntersectionOfLines (leftLine, defenderLine, leftPoint))
    {

    }
    if (Geometry::getIntersectionOfLines (rightLine, defenderLine, rightPoint))
    {

    }
    return leftPoint;
//    if(ball.suitable.y() >= 0)
//    {
//      usingPoint = rightPoint;
//    }
//    else
//    {
//      usingPoint = leftPoint;
//    }
//    defenderPosition = (usingPoint + intersectionOfMainAngleBisectorAndDefenderLine) / 2;
//    return defenderPosition;
  }

//for supporter
  Vector2f
  LibDefender::supporterPositionWhenKeeperNotPenalized ()
  {
    Vector2f defenderPosition;
//    Vector2f intersectionOfMainAngleBisectorAndDefenderLine = defenderPositionWhenKeeperPenalized();
//    Vector2f intersectionOfLeftLineAndDefenderLine;
//    Vector2f intersectionOfRightLineAndDefenderLine;
    Vector2f leftPoint;
    Vector2f rightPoint;
    Vector2f usingPoint;
    Vector2f goalLeft = Vector2f (theFieldDimensions.xPosOwnGroundline,
				  theFieldDimensions.yPosLeftGoal);
    Vector2f goalRight = Vector2f (theFieldDimensions.xPosOwnGroundline,
				   theFieldDimensions.yPosRightGoal);
    Vector2f leftLineDirection = Vector2f (
	ball.suitable.x () - theFieldDimensions.xPosOwnGroundline,
	ball.suitable.y () - theFieldDimensions.yPosLeftGoal);
    Vector2f rightLineDirection = Vector2f (
	ball.suitable.x () - theFieldDimensions.xPosOwnGroundline,
	ball.suitable.y () - theFieldDimensions.yPosRightGoal);
    Geometry::Line leftLine = Geometry::Line (goalLeft, leftLineDirection);
    Geometry::Line rightLine = Geometry::Line (goalRight, rightLineDirection);
    if (Geometry::getIntersectionOfLines (leftLine, defenderLine, leftPoint))
    {

    }
    if (Geometry::getIntersectionOfLines (rightLine, defenderLine, rightPoint))
    {

    }
    return leftPoint;
    /*
     if(ball.suitable.y() >= 0)
     {
     usingPoint = rightPoint;
     }
     else
     {
     usingPoint = leftPoint;
     }
     defenderPosition = (usingPoint + intersectionOfMainAngleBisectorAndDefenderLine) / 2;
     return defenderPosition; */
  }

  Pose2f
  LibDefender::defenderDefencePosition ()
  {
    Pose2f defencePosition;
    Vector2f defenderPositionWhenNotKeeperPenalizedRelative (-3200, -900);
    float angle = ballPositionRobot.angle ();
    angle = angle > (0.52f - robot.rotation) ? (0.52f - robot.rotation) : angle;
    angle = angle < (-0.52f - robot.rotation) ? (-0.52f - robot.rotation) : angle;
//    bool leftGoal = false;
//    if (robot.y>300.f)
//      leftGoal = true;
//  Vector2f ballSuitablePositionRelative = Geometry::fieldCoord2Relative(
//      Pose2f(robot.rotation, robot.x, robot.y),
//      Vector2f(ball.suitable.x() - 150.f, ball.suitable.y()));

//    Vector2f defenderPositionWhenKeeperPenalizedRelative = Geometry::fieldCoord2Relative(Pose2f(robot.rotation, robot.x, robot.y), defenderPositionWhenKeeperPenalized());
    Vector2f BallToLeftGoal = Vector2f(ball.teamPosition - Vector2f(-4500.f, 800.f));
    Vector2f BallToRightGoal = Vector2f(ball.teamPosition - Vector2f(-4500.f, -800.f));
    Vector2f defenderPositionLeftGoal = Geometry::fieldCoord2Relative (Pose2f (robot.rotation, robot.x, robot.y),
														defenderPositionCoorperateWithKeeperAndSupporter (true));
    Vector2f defenderPositionRightGoal = Geometry::fieldCoord2Relative (Pose2f (robot.rotation, robot.x, robot.y),
														defenderPositionCoorperateWithKeeperAndSupporter (false));
    if (defenderPositionRightGoal.norm () < defenderPositionLeftGoal.norm ())
    {
		defenderPositionWhenNotKeeperPenalizedRelative = defenderPositionRightGoal;
    }
    else
    {
		defenderPositionWhenNotKeeperPenalizedRelative = defenderPositionLeftGoal;
    }
//    Vector2f defenderPositionWhenNotKeeperPenalizedRelative =
////        Geometry::fieldCoord2Relative(Pose2f(robot.rotation, robot.x, robot.y),
////                                      defenderPositionWhenKeeperNotPenalized());
//	Geometry::fieldCoord2Relative (
//	    Pose2f (robot.rotation, robot.x, robot.y),
//	    defenderPositionCoorperateWithKeeperAndSupporter (leftGoal));
//    if(teammate.isKeeperPenalized == true)
//      defencePosition = Pose2f(angle, defenderPositionWhenKeeperPenalizedRelative.x(), defenderPositionWhenKeeperPenalizedRelative.y());
//    else
	if (ballPositionField.x() > 750.f)
	{
		Vector2f dfp(-3200, -900);
		Vector2f asideLeftGoal;
		Vector2f asideRightGoal;
		asideLeftGoal = Geometry::fieldCoord2Relative(Pose2f(robot.rotation, robot.x, robot.y),Vector2f(-3200.f, 900.f));
		asideRightGoal = Geometry::fieldCoord2Relative(Pose2f(robot.rotation, robot.x, robot.y),Vector2f(-3200.f, -900.f));
		if(asideLeftGoal.norm() < asideRightGoal.norm())
		{
			dfp = asideLeftGoal;
		}
		else
		{
			dfp = asideRightGoal;
		}
		defencePosition = Pose2f(angle, dfp.x(), dfp.y());
	}
	else
		defencePosition = Pose2f(angle,defenderPositionWhenNotKeeperPenalizedRelative.x(),defenderPositionWhenNotKeeperPenalizedRelative.y());
	return defencePosition;
  }

//for supporter
  Pose2f
  LibDefender::supporterDefencePosition ()
  {
    Pose2f defencePosition;
    float angle = ball.angleRad;
    angle = angle > (0.52f - robot.rotation) ? (0.52f - robot.rotation) : angle;
    angle =
	angle < (-0.52f - robot.rotation) ? (-0.52f - robot.rotation) : angle;
    //Vector2f supporterPositionWhenKeeperPenalizedRelative = Geometry::fieldCoord2Relative(Pose2f(robot.rotation, robot(), robot.y), defenderPositionWhenKeeperPenalized());
    Vector2f supporterPositionWhenNotKeeperPenalizedRelative =
	Geometry::fieldCoord2Relative (
	    Pose2f (robot.rotation, robot.x, robot.y),
	    supporterPositionWhenKeeperNotPenalized ());
//	  if(theTeammateData.isPenalized[1] == true)
//		  defencePosition = Pose2f(angle, defenderPositionWhenKeeperPenalizedRelative.x(), defenderPositionWhenKeeperPenalizedRelative.y();
//	  else
    defencePosition = Pose2f (
	angle, supporterPositionWhenNotKeeperPenalizedRelative.x (),
	supporterPositionWhenNotKeeperPenalizedRelative.y ());
    return defencePosition;
  }

  Vector2f
  LibDefender::globalSpeed (Vector2f relativeSpeed)
  {
    Vector2f speed;
//	  Vector2f startPointOfSpeedVectorGlobal = Geometry::relative2FieldCoord(Pose2f(robot.rotation, robot.x, robot.y), Vector2f(robot.x, robot.y) + relativeSpeed);
//	  Vector2f endPointOfSpeedVectorGlobal = Geometry::relative2FieldCoord(Pose2f(robot.rotation, robot.x, robot.y), relativeSpeed);
//	  speed = endPointOfSpeedVectorGlobal - startPointOfSpeedVectorGlobal;
    speed.x () = float (
	relativeSpeed.x () * cos (robot.rotation)
	    - relativeSpeed.y () * sin (robot.rotation));
    speed.y () = float (
	relativeSpeed.x () * sin (robot.rotation)
	    + relativeSpeed.y () * cos (robot.rotation));
    return speed;
  }

  float
  LibDefender::yIfBallCanPassGroundLine ()
  {
    ball.getSuitablePosition ();
    Vector2f speed = globalSpeed (ball.speedRelative);
    if (speed.x () == 0 || speed.x () > -0.5f || ball.global.x () < robot.x)
    {
      return 5000.f;
    }
    float timeWhenAxisReached = (fabs (
	ball.suitable.x () - theFieldDimensions.xPosOwnGroundline) / speed.x ());

    Vector2f finalBallPosition = ball.suitable + (speed * timeWhenAxisReached);
    return finalBallPosition.y ();
  }

  bool
  LibDefender::ifStopBallNeeded ()
  {
    if (ball.speedRelative.x () > 0 || ball.speedRelative.x () == 0)
    {
      return false;
    }
    else
    {
      if (yIfBallCanPassGroundLine () > (theFieldDimensions.yPosLeftGoal + 100)
	  || yIfBallCanPassGroundLine ()
	      < (theFieldDimensions.yPosRightGoal - 100))
	return false;
      else
	return true;
    }
  }
  bool
  LibDefender::ifStopBallNeeded2 ()
  {
    if (ballSpeedRobot.x () > 0 || ballSpeedRobot.x () == 0)
    {
      return false;
    }
    else
    {
      if (abs (yIfBallPassOwnAxis) > 800.f || ballEndPositionRobot.x () > 200.f)
		return false;
	  else
		return true;
    }
  }

  bool
  LibDefender::toStopBall ()
  {

    if (ifStopBallNeeded () && robotAndBallCloseEnough ())
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool
  LibDefender::toStopBall2 ()
  {
    float timeWhenBallReached = 1000;
    if (ballPositionRobot.x () > 0)
      timeWhenBallReached = BallPhysics::timeForDistance (
	  Vector2f (ballSpeedRobot.x (), 0), ballPositionRobot.x (),
	  theFieldDimensions.ballFriction);
    if (ifStopBallNeeded2 ()
	&& (ball.positionRobot.norm() < 1500.f || (timeWhenBallReached < 1.f && ball.positionRobot.norm() < 2000.f)))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool
  LibDefender::ifSpreadNeeded ()
  {
    if (!toStopBall ())
    {
      return false;
    }
    else
    {
      if ((ball.yPosWhenBallReachesOwnYAxis () > 100
	  && ball.yPosWhenBallReachesOwnYAxis () < 600)
	  || (ball.yPosWhenBallReachesOwnYAxis () < -100
	      && ball.yPosWhenBallReachesOwnYAxis () > -600))
      {
	if (ball.timeWhenAxisReached < 800.f && ball.timeWhenAxisReached > 100)
	  return true;
	else
	  return false;
      }

      else
	return false;
    }
  }

  bool
  LibDefender::ifSpreadNeeded2 ()
  {
    if (!toStopBall2 ())
    {
      return false;
    }
    else
    {
      if ((yIfBallPassOwnAxis > 50 && yIfBallPassOwnAxis < 500)
	  || (yIfBallPassOwnAxis < -50 && yIfBallPassOwnAxis > -500))
      {
	if (ball.timeWhenAxisReached < 1.5f && ball.timeWhenAxisReached > 0)
	  return true;
	else
	  return false;
      }

      else
	return false;
    }
  }

  bool
  LibDefender::robotAndBallCloseEnough ()
  {
    ball.getSuitablePosition ();
    bool closeEnough = false;
    if ((ball.suitable.x () - robot.x) * (ball.suitable.x () - robot.x)
	+ (ball.suitable.y () - robot.y) * (ball.suitable.y () - robot.y)
	< 1000 * 1000 || ball.positionRobot.x() * ball.positionRobot.x() + ball.positionRobot.y() * ball.positionRobot.y() < 1000 * 1000)
      closeEnough = true;
    else
      closeEnough = false;
    return closeEnough;
  }
  bool
  LibDefender::robotAndBallCloseEnough2 ()
  {
    bool closeEnough = false;
    if ((ballPositionField.x () - robot.x) * (ballPositionField.x () - robot.x)
	+ (ballPositionField.y () - robot.y)
	    * (ballPositionField.y () - robot.y) < 1500 * 1500
	|| ballPositionRobot.x () * ballPositionRobot.x ()
	    + ballPositionRobot.y () * ballPositionRobot.y () < 1500 * 1500)
      closeEnough = true;
    else
      closeEnough = false;
    return closeEnough;
  }

  Vector2f
  LibDefender::getBallPositionField ()
  {
	Vector2f ownSeen = ball.global;
	useTeamBall = false;
	if (ball.wasSeen() || ball.notSeenTime() < 1000) {
		useTeamBall = false;
		return ownSeen;
	}
	else{
		if (theTeamBallModel.isValid)
		{
			useTeamBall = true;
			return theTeamBallModel.position;
		}
		else
			return ownSeen;
	}
	return ownSeen;
  }

  /**
   * @function get ball position to robot using both own observation and team info
   * @return ball poition to robot
   */
  Vector2f
  LibDefender::getBallPositionRobot ()
  {
    return Transformation::fieldToRobot (theRobotPose, ballPositionField);
  }

  /**
   * @function get ball speed on field
   * @return ball speed on field
   */
  Vector2f
  LibDefender::getBallSpeedField ()
  {
    if (useTeamBall)
      return theTeamBallModel.velocity;
    else
      return Transformation::robotToField (theRobotPose, ball.speedRelative);
  }
  
  /**
   * @function get ball speed to robot
   * @return ball speed to robot
   */
  Vector2f
  LibDefender::getBallSpeedRobot ()
  {
    return Transformation::fieldToRobot (theRobotPose, ballSpeedField);
  }
  
  /**
   * @function get ball end point on field
   * @return ball end point on field
   */
  Vector2f
  LibDefender::getBallEndPointField ()
  {
    return Transformation::robotToField (theRobotPose, ballEndPositionRobot);
  }
  
  /**
   * @function get ball end point to robot
   * @return ball end point to robot
   */
  Vector2f
  LibDefender::getBallEndPointRobot ()
  {
    return BallPhysics::getEndPosition (ballPositionRobot, ballSpeedRobot,
					theFieldDimensions.ballFriction);
  }

  float
  LibDefender::getYIfBallPassOwnAxis ()
  {
    Geometry::Line BallToRobot = Geometry::Line (ballPositionRobot,
						 ballSpeedRobot);
    Vector2f intersection;
    if (!Geometry::getIntersectionOfLines (
	BallToRobot, Geometry::Line (Vector2f (0, 0), Vector2f (0, 1)),
	intersection))
      return 5000.f;
    return intersection.y ();
  }

  Pose2f LibDefender::defenderDefencePosition2()
    {
      Pose2f defencePosition;
      Vector2f defenderPositionWhenNotKeeperPenalizedRelative (-3200, -900);
      float angle = ballPositionRobot.angle ();
      angle = angle > (0.52f - robot.rotation) ? (0.52f - robot.rotation) : angle;
      angle = angle < (-0.52f - robot.rotation) ? (-0.52f - robot.rotation) : angle;
      Vector2f defenderPositionLeftGoal = Geometry::fieldCoord2Relative (Pose2f (robot.rotation, robot.x, robot.y),
  														defenderPositionCoorperateWithKeeperAndSupporter (true));
      Vector2f defenderPositionRightGoal = Geometry::fieldCoord2Relative (Pose2f (robot.rotation, robot.x, robot.y),
  														defenderPositionCoorperateWithKeeperAndSupporter (false));
		if (ball.teamPosition.y() < -500.f)
		{
			defenderPositionWhenNotKeeperPenalizedRelative = defenderPositionRightGoal;
			lastIsLeft = false;
		}
		else if(ball.teamPosition.y() > 500.f)
		{
			defenderPositionWhenNotKeeperPenalizedRelative = defenderPositionLeftGoal;
			lastIsLeft = true;
		}
		else
		{
			defenderPositionWhenNotKeeperPenalizedRelative = Geometry::fieldCoord2Relative (Pose2f (robot.rotation, robot.x, robot.y),
															defenderPositionCoorperateWithKeeperAndSupporter (lastIsLeft));
		}

	//	if (ballPositionField.x() > 750.f)
	//	{
	//		Vector2f dfp(-3200, -900);
	//		Vector2f asideLeftGoal;
	//		Vector2f asideRightGoal;
	//		asideLeftGoal = Geometry::fieldCoord2Relative(Pose2f(robot.rotation, robot.x, robot.y),Vector2f(-3200.f, 900.f));
	//		asideRightGoal = Geometry::fieldCoord2Relative(Pose2f(robot.rotation, robot.x, robot.y),Vector2f(-3200.f, -900.f));
	//		if(asideLeftGoal.norm() < asideRightGoal.norm())
	//		{
	//			dfp = asideLeftGoal;
	//		}
	//		else
	//		{
	//			dfp = asideRightGoal;
	//		}
	//		defencePosition = Pose2f(angle, dfp.x(), dfp.y());
	//	}
	//	else
	//		defencePosition = Pose2f(angle,defenderPositionWhenNotKeeperPenalizedRelative.x(),defenderPositionWhenNotKeeperPenalizedRelative.y());
		defencePosition = Pose2f(angle, defenderPositionWhenNotKeeperPenalizedRelative);
		return defencePosition;
	}
}
