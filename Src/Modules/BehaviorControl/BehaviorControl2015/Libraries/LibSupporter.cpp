#include "../LibraryBase.h"

namespace Behavior2015
{
#include "LibBall.h"
#include "LibSupporter.h"
#include "LibDefender.h"
#include "LibKeeper.h"
#include "LibRobot.h"
#include "LibTeammate.h"

  LibSupporter::LibSupporter ()
  {
    currentIntention = SupporterIntention::NONE;
    preIntention = SupporterIntention::NONE;
    useTeamBall = false;
    wrongLacation = false;
    
  }

  void
  LibSupporter::preProcess ()
  {
//    ballPosition = getBallAbsolutePosition();
//    ballPositionRelative = Geometry::fieldCoord2Relative(
//      Pose2f(robot.rotation, robot.x, robot.y),
//      Vector2f(ballPosition.x() - 150.f, ballPosition.y()));
//    ballSpeedRelative = getBallSpeedRelative();
    yIfBallPassOwnAxis = getYIfBallPassOwnAxis();
  }

  void
  LibSupporter::postProcess ()
  {
  }

  Vector2f
  LibSupporter::ballPositionFromKeeper ()
  {
    return teammate.ballGlobalFromRobot(robot.numKeeper);
  }

  Vector2f
  LibSupporter::ballPositionFromDefender ()
  {
    return teammate.ballGlobalFromRobot(robot.numDefender);
  }

  Vector2f
//  LibSupporter::supporterStandPosition ()
//  {
//    Vector2f standPosition(0.f, 0.f);
//    Pose2f DefenderPose = teammate.robotPose(robot.numDefender); //later can be replaced by Current_Defender_Pose
//    Teammate::Status DefenderState = teammate.robotState(robot.numDefender);
////  ball.getSuitablePosition();
//    Vector2f goalCenter(-4500, 0);
//    Vector2f intersectionLeft, intersectionRight, intersection;
//    float CalY = yPoseCoorperateWithStrikerAndStabber();
//    float CalX = teammate.getSupporterPose().translation.x();
//    if (ball.positionField.x() > 0)
//    {
//    if (robot.x < -3750.f)
//    	standPosition.x() = -3200.f;
//    else if (robot.x < -2650.f)
//    {
//        if (ball.positionField.x()>500)
//            standPosition.x() = -2100.f;
//        else
//        {
//            if (abs(standPosition.x()+2100)<100.f)
//            {
//                if (ball.positionField.x()<-50.f)
//                    standPosition.x() = -3200.f;
//                else
//                    standPosition.x() = -2100.f;
//            }
//            else
//              standPosition.x() = -3200.f;
//        }
//
//    }
//    else if (robot.x < -1550.f)
//    {
//        if (ball.positionField.x()<-200)
//            standPosition.x() = -3200.f;
//        else if (ball.positionField.x()<2000.f)
//        {
//            if (abs(standPosition.x()+3200)<100.f)
//            {
//                if (ball.positionField.x()>500.f)
//                    standPosition.x() = -2100.f;
//                else
//                    standPosition.x() = -3200.f;
//            }
//            else if (abs(standPosition.x()+1000)<100.f)
//            {
//                if (ball.positionField.x()<1500.f)
//                    standPosition.x() = -2100.f;
//                else
//                    standPosition.x() = -1000.f;
//            }
//            else
//                standPosition.x() = -2100.f;
//        }
//        else
//            standPosition.x() = -1000.f;
//    }
//
//    else if (robot.x < -450.f)
//    {
//        if (ball.positionField.x()<1200)
//            standPosition.x() = -2100.f;
//        else if (ball.positionField.x()<3300.f)
//        {
//            if (abs(standPosition.x()+2100)<100.f)
//            {
//                if (ball.positionField.x()>1800.f)
//                    standPosition.x() = -1000.f;
//                else
//                    standPosition.x() = -2100.f;
//            }
//            else if (abs(standPosition.x()+0)<100.f)
//            {
//                if (ball.positionField.x()<3000.f)
//                    standPosition.x() = -1000.f;
//                else
//                    standPosition.x() = 0.f;
//            }
//            else
//                standPosition.x() = -1000.f;
//        }
//        else
//            standPosition.x() = 0.f;
//    }
//    else
//    {
//      standPosition.x() = 0.f;
//    }
////      if (abs(ball.positionField.x() - 2000) < 500.f)
////	standPosition.x() = -1600.f;
////      else
////	standPosition.x() = 0;
//
////    standPosition.x() = static_cast<int>(ballPosition.x() / 1000.f) * 800.f
////        - 2400.f;
//      if (CalY > 3000)
//      {
//	Vector2f lineDirection = Vector2f(
//	    ball.positionField.x() - goalCenter.x(),
//	    ball.positionField.y() - goalCenter.y());
//	Geometry::Line BallAndGoalCenter = Geometry::Line(ball.positionField,
//							  lineDirection);
//	if (Geometry::getIntersectionOfLines(
//	    BallAndGoalCenter,
//	    Geometry::Line(Vector2f(standPosition.x(), 0), Vector2f(0, 1)),
//	    intersection))
//	  standPosition.y() = intersection.y();
//	else
//	  standPosition.y() = 0;
//      }
//      else
//	standPosition.y() = CalY;
//      // now we choose a suitble standPosition.x()
//
////     standPosition.x() = CalX;
//
//    }
//    else
//    {
//      standPosition.x() = -3200;
//      Vector2f goalLeft = Vector2f(theFieldDimensions.xPosOwnGroundline,
//				   theFieldDimensions.yPosLeftGoal);
//      Vector2f goalRight = Vector2f(theFieldDimensions.xPosOwnGroundline,
//				    theFieldDimensions.yPosRightGoal);
//      Vector2f lineDirection2Left = Vector2f(
//	  ball.positionField.x() - goalLeft.x(),
//	  ball.positionField.y() - goalLeft.y());
//      Vector2f lineDirection2Right = Vector2f(
//	  ball.positionField.x() - goalRight.x(),
//	  ball.positionField.y() - goalRight.y());
//      Geometry::Line BallAndGoalLeft = Geometry::Line(ball.positionField,
//						      lineDirection2Left);
//      Geometry::Line BallAndGoalRight = Geometry::Line(ball.positionField,
//						       lineDirection2Right);
//
//      if (!Geometry::getIntersectionOfLines(
//	  BallAndGoalLeft, Geometry::Line(Vector2f(-3200, 0), Vector2f(0, 1)),
//	  intersectionLeft)
//	  || !Geometry::getIntersectionOfLines(
//	      BallAndGoalRight,
//	      Geometry::Line(Vector2f(-3200, 0), Vector2f(0, 1)),
//	      intersectionRight))
//	standPosition.y() = 0;
//      if (DefenderState != Teammate::INACTIVE)
//      {
//	if ((intersectionLeft - DefenderPose.translation).norm()
//	    < (intersectionRight - DefenderPose.translation).norm())
//	  standPosition.y() = intersectionRight.y();
//	else
//	  standPosition.y() = intersectionLeft.y();
//      }
//      else
//      {
//	if ((intersectionLeft - robot.pose.translation).norm()
//	    < (intersectionRight - robot.pose.translation).norm())
//	  standPosition.y() = intersectionLeft.y();
//	else
//	  standPosition.y() = intersectionRight.y();
//      }
//    }
//    if (DefenderState != Teammate::INACTIVE)
//    {
//      if (abs(standPosition.x() - DefenderPose.translation.x()) < 1000.f)
//      {
//	if (standPosition.y() >= DefenderPose.translation.y()
//	    && standPosition.y() < (DefenderPose.translation.y() + 800.f))
//	  standPosition.y() = DefenderPose.translation.y() + 800.f;
//	if (standPosition.y() < DefenderPose.translation.y()
//	    && standPosition.y() > (DefenderPose.translation.y() - 800.f))
//	  standPosition.y() = DefenderPose.translation.y() - 800.f;
//      }
//    }
////    std::cout << "standPosition.x: " << standPosition.x() << " standPosition.y: " << standPosition.y() << "ball.positionField.x" << ball.positionField.x() << std::endl;
//    return standPosition;
//  }

  LibSupporter::supporterStandPosition ()
  {
    Vector2f standPosition(0.f, 0.f);
    Pose2f DefenderPose = teammate.robotPose(robot.numDefender); //later can be replaced by Current_Defender_Pose
    Teammate::Status DefenderState = teammate.robotState(robot.numDefender);
    Vector2f goalCenter(-4500, 0);
    Vector2f intersectionLeft, intersectionRight, intersection;
    float CalY = yPoseCoorperateWithStrikerAndStabber();
    float CalX = teammate.getSupporterPose().translation.x();
    if (ball.positionField.x() >= -20.f)
    {

    	if (robot.x <= -3750.f)
    	{
    		standPosition.x() = -3200.f;
    	}
    	if (abs(robot.x + 3200.f) < 550.f || robot.x == 2650.f)
    	{
    		if (ball.endPositionField.x() > ball.positionField.x())
    		{
    			if (ball.positionField.x() < 500.f)
    				standPosition.x() = -3200.f;
    			else
    				standPosition.x() = -2100.f;
    		}
    		else
    		{
    			standPosition.x() = -2100.f;
    		}
    	}
    	if (abs(robot.x + 2100) < 550 || robot.x == -1550)
    	{
    		if (ball.endPositionField.x() > ball.positionField.x())
    		{
				if (ball.positionField.x() < 500.f)
					standPosition.x() = -2100.f;
				else
					standPosition.x() = -1100.f;
    		}
    		else
    		{
    			standPosition.x() = -1100.f;
    		}
    	}
    	else
    		standPosition.x() = -1100.f;

//      if (abs(ball.positionField.x() - 2000) < 500.f)
//	standPosition.x() = -1600.f;
//      else
//	standPosition.x() = 0;

//    standPosition.x() = static_cast<int>(ballPosition.x() / 1000.f) * 800.f
//        - 2400.f;
//      if (CalY > 3000)
//      {
	Vector2f lineDirection = Vector2f(
	    ball.positionField.x() - goalCenter.x(),
	    ball.positionField.y() - goalCenter.y());
	Geometry::Line BallAndGoalCenter = Geometry::Line(ball.positionField,
							  lineDirection);
	if (Geometry::getIntersectionOfLines(
	    BallAndGoalCenter,
	    Geometry::Line(Vector2f(standPosition.x(), 0), Vector2f(0, 1)),
	    intersection))
	  standPosition.y() = intersection.y();
//	else
//	  standPosition.y() = 0;
//      }
//      else
//	standPosition.y() = CalY;
      // now we choose a suitble standPosition.x()

//     standPosition.x() = CalX;

    }
    else
    {
      standPosition.x() = -3200;
      Vector2f goalLeft = Vector2f(theFieldDimensions.xPosOwnGroundline,
				   theFieldDimensions.yPosLeftGoal);
      Vector2f goalRight = Vector2f(theFieldDimensions.xPosOwnGroundline,
				    theFieldDimensions.yPosRightGoal);
      Vector2f lineDirection2Left = Vector2f(
	  ball.positionField.x() - goalLeft.x(),
	  ball.positionField.y() - goalLeft.y());
      Vector2f lineDirection2Right = Vector2f(
	  ball.positionField.x() - goalRight.x(),
	  ball.positionField.y() - goalRight.y());
      Geometry::Line BallAndGoalLeft = Geometry::Line(ball.positionField,
						      lineDirection2Left);
      Geometry::Line BallAndGoalRight = Geometry::Line(ball.positionField,
						       lineDirection2Right);

      if (!Geometry::getIntersectionOfLines(
	  BallAndGoalLeft, Geometry::Line(Vector2f(-3200, 0), Vector2f(0, 1)),
	  intersectionLeft)
	  || !Geometry::getIntersectionOfLines(
	      BallAndGoalRight,
	      Geometry::Line(Vector2f(-3200, 0), Vector2f(0, 1)),
	      intersectionRight))
	standPosition.y() = 0;
      if (DefenderState != Teammate::INACTIVE)
      {
	if ((intersectionLeft - DefenderPose.translation).norm()
	    < (intersectionRight - DefenderPose.translation).norm())
	  standPosition.y() = intersectionRight.y();
	else
	  standPosition.y() = intersectionLeft.y();
      }
      else
      {
	if ((intersectionLeft - robot.pose.translation).norm()
	    < (intersectionRight - robot.pose.translation).norm())
	  standPosition.y() = intersectionLeft.y();
	else
	  standPosition.y() = intersectionRight.y();
      }
    }
    if (DefenderState != Teammate::INACTIVE)
    {
      if (abs(standPosition.x() - DefenderPose.translation.x()) < 1000.f)
      {
	if (standPosition.y() >= DefenderPose.translation.y()
	    && standPosition.y() < (DefenderPose.translation.y() + 800.f))
	  standPosition.y() = DefenderPose.translation.y() + 800.f;
	if (standPosition.y() < DefenderPose.translation.y()
	    && standPosition.y() > (DefenderPose.translation.y() - 800.f))
	  standPosition.y() = DefenderPose.translation.y() - 800.f;
      }
    }
//    std::cout << "standPosition.x: " << standPosition.x() << " standPosition.y: " << standPosition.y() << "ball.positionField.x: " << ball.positionField.x() << std::endl;
    return standPosition;
  }

  float
  LibSupporter::yPoseCoorperateWithStrikerAndStabber ()
  {
    float y = 0.f;
    Teammate::Status StrikerState = teammate.robotState(robot.numStriker);
    Teammate::Status StabberState = teammate.robotState(robot.numStabber);
    Pose2f StrikerPose = teammate.robotPose(robot.numStriker);
    Pose2f StabberPose = teammate.robotPose(robot.numStabber);
    Pose2f anotherRobotPose;
    if (StrikerState == Teammate::INACTIVE
	&& StabberState == Teammate::INACTIVE)
    {
      return 5000.f;
    }

    else if (StrikerState != StabberState)
    {

      if (StrikerState != Teammate::INACTIVE)
      {
	anotherRobotPose = StrikerPose;
      }
      else
	anotherRobotPose = StabberPose;
    }
    else
    {
      if ((ball.positionField.y() < StrikerPose.translation.y()
	  && ball.positionField.y() > StabberPose.translation.y())
	  || (ball.positionField.y() > StrikerPose.translation.y()
	      && ball.positionField.y() < StabberPose.translation.y()))
	anotherRobotPose.translation.y() = (StrikerPose.translation.y()
	    + StabberPose.translation.y()) / 2.f;
      else
	anotherRobotPose.translation.y() = ball.positionField.y();
    }
    y = static_cast<int>((static_cast<int>(anotherRobotPose.translation.y()
	/ 375.f) + 1) / 2) * 750;
    if (y < -2000)
      y = -2000;
    if (y > 2000)
      y = 2000;
    return y;
  }

  SupporterIntention
  LibSupporter::NextIntentionDeterminedByCurrent (SupporterIntention it,
						  Vector2f ballSpeedRelative,
						  Vector2f positionAbsolute,
						  Vector2f endPosition,
						  Vector2f endPositionRelative)
  {
    switch (it)
    {
      case SupporterIntention::WALK_TO_DEFEND_LINE:
	if (positionAbsolute.x() > dangerLine
	    && (ball.speedRobot.x() > robotSpeedX || endPosition.x() > -750))
	  return SupporterIntention::WALK_TO_HALF_FIELD;
	else
	  return SupporterIntention::WALK_TO_DEFEND_LINE;
      case SupporterIntention::WALK_TO_HALF_FIELD:
	if ((endPosition.x() < -1000 && ball.speedRobot.x() < 0)
	    || positionAbsolute.x() < -1200)
	  return SupporterIntention::WALK_TO_DEFEND_LINE;
	else if (positionAbsolute.x() > 3000.f
	    || (endPosition.x() > 2500.f || ball.speedRobot.x() > robotSpeedX))
	  return SupporterIntention::WALK_TO_CENTER_CIRCLE;
	else
	  return SupporterIntention::WALK_TO_HALF_FIELD;
      case SupporterIntention::WALK_TO_CENTER_CIRCLE:
	if (endPosition.x() < 2700 && abs(endPositionRelative.y()) > 1500.f)
	  return SupporterIntention::WALK_TO_HALF_FIELD;
	else
	  return SupporterIntention::WALK_TO_CENTER_CIRCLE;
      default:
	return SupporterIntention::NONE;
    }
  }

  Vector2f
  LibSupporter::SupportPositionDeterminedByIntention (SupporterIntention it)
  {
    Vector2f target;
    Pose2f DefenderPose = teammate.robotPose(4); //later can be replaced by Current_Defender_Pose
    Teammate::Status DefenderState = teammate.robotState(4);
//    std::cout<<int(it)<<std::endl;
    switch (it)
    {
      case SupporterIntention::WALK_TO_DEFEND_LINE:
	target.x() = -3200;
	break;
      case SupporterIntention::WALK_TO_HALF_FIELD:
	target.x() = -2000;
	break;
      case SupporterIntention::WALK_TO_CENTER_CIRCLE:
	target.x() = 0;
	break;
      default:
	target.x() = -3000;
	break;
    }
    Vector2f goalLeft = Vector2f(theFieldDimensions.xPosOwnGroundline,
				 theFieldDimensions.yPosLeftGoal);
    Vector2f goalRight = Vector2f(theFieldDimensions.xPosOwnGroundline,
				  theFieldDimensions.yPosRightGoal);
    Vector2f goal;
    goal = goalLeft;
    if (DefenderState != Teammate::INACTIVE)
    {
      if (goal == goalLeft && DefenderPose.translation.y() < 200)
	goal = goalRight;
      if (goal == goalRight && DefenderPose.translation.y() > -200)
	goal = goalLeft;
    }
    else
    {
      if (goal == goalLeft && robot.y < 200)
	goal = goalRight;
      if (goal == goalRight && robot.y > -200)
	goal = goalLeft;
    }

    Vector2f lineDirection = Vector2f(ball.positionField.x() - goal.x(),
				      ball.positionField.y() - goal.y());
    Vector2f intersection;
    Geometry::Line BallAndGoalCenter = Geometry::Line(ball.positionField,
						      lineDirection);
    if (Geometry::getIntersectionOfLines(
	BallAndGoalCenter,
	Geometry::Line(Vector2f(target.x(), 0), Vector2f(0, 1)), intersection))
      target.y() = intersection.y();
    else
      target.y() = 0;
    std::cout << "x=" << target.x() << '\t' << "y=" << target.y() << std::endl;
    return target;
  }

  Vector2f
  LibSupporter::getBallSpeedRelative ()
  {
    if (!useTeamBall)
      return ball.speedRelative;
    else
      return Geometry::fieldCoord2Relative(theRobotPose,
					   theTeamBallModel.velocity);
  }

  Vector2f
  LibSupporter::getBallEndPoint ()
  {
    if (!useTeamBall)
      return ball.endPosition;
    else
      return BallPhysics::getEndPosition(ball.positionRobot,
					 getBallSpeedRelative(),
					 theFieldDimensions.ballFriction);
  }

  Vector2f
  LibSupporter::getBallEndPointRelative ()
  {
    return Geometry::relative2FieldCoord(theRobotPose, getBallEndPoint());
  }

  Vector2f
  LibSupporter::getBallAbsolutePosition ()
  {
    wrongLacation = false;
    Vector2f seenByItself = ball.global;
    /**if team ball is invalid or team communication crash
     * supporter use the information observered itself
     * */
    if (!theLocalizationTeamBall.isValid)
    {
      useTeamBall = false;
      return seenByItself;
      if (!ball.wasSeen())
	return Vector2f(5000, 5000);
    }

    else
    {
      /**if goalKeeper saw the ball, and its sideConfidence
       * is very good, calculate the error
       * */
      if (theLocalizationTeamBall.goalieHasObserved)
      {
	/** if the error is compatible with team ball
	 * return the information observered itself
	 * */
	float errorThresold = 1000.f;
	if (ball.wasSeen())
	{
	  Vector2f error = seenByItself - theLocalizationTeamBall.position;
	  if (error.norm() < errorThresold)
	  {
	    useTeamBall = false;
	    return seenByItself;
	  }

	  /** if the error is too large,
	   * take the information from defender into consideration.
	   * if defender says teamball is right,
	   * return teamball information.
	   * */
	  Vector2f errorDefender = ballPositionFromDefender()
	      - theLocalizationTeamBall.position;
	  if (errorDefender.norm() < errorThresold)
	  {
	    useTeamBall = true;
	    return theLocalizationTeamBall.position;
	  }
	  if (theLocalizationTeamBall.numOfObservers >= 2)
	  {
	    useTeamBall = true;
	    return theLocalizationTeamBall.position;
	  }

	}
	else
	{
	  if (theLocalizationTeamBall.numOfObservers >= 1)
	  {
	    useTeamBall = true;
	    return theLocalizationTeamBall.position;
	  }
	  else
	  {
	    useTeamBall = false;
	    return seenByItself;

	  }
	}
      }
      else
      {
	if (!ball.wasSeen() && ball.notSeenTime() > 5000.f)
	{
	  useTeamBall = true;
	  return theLocalizationTeamBall.position;
	}
	else
	{
	  /** only when enough teammates saw the ball
	   * can  the information be considered
	   * */
	  if (theLocalizationTeamBall.numOfObservers >= 2)
	  {
	    float errorThresold = 1000.f;
	    Vector2f error = seenByItself - theLocalizationTeamBall.position;
	    if (error.norm() < errorThresold)
	    {
	      useTeamBall = false;
	      return seenByItself;
	    }

	    else
	    {
	      if (theLocalizationTeamBall.numOfObservers > 2)
	      {
		useTeamBall = true;
		return theLocalizationTeamBall.position;
	      }
	      if (error.norm() < 2 * errorThresold)
	      {
		useTeamBall = true;
		return theLocalizationTeamBall.position;

	      }
	      else
	      {
		useTeamBall = false;
		return seenByItself;
	      }
	    }
	  }
	}
      }
    }
  }

  Vector2f
  LibSupporter::defenderPositionWhenKeeperPenalized ()
  {
    Vector2f intersectionOfAngleBisectorAndDefenderLine;
    ball.getSuitablePosition();
    Vector2f goalLeft = Vector2f(theFieldDimensions.xPosOwnGroundline,
				 theFieldDimensions.yPosLeftGoal);
    Vector2f goalRight = Vector2f(theFieldDimensions.xPosOwnGroundline,
				  theFieldDimensions.yPosRightGoal);
    float goalWidth = theFieldDimensions.yPosLeftGoal
	- theFieldDimensions.yPosRightGoal;
    float distanceBallToGoalLeft = (goalLeft - ball.positionField).norm();
    float distanceBallToGoalRight = (goalRight - ball.positionField).norm();
    yOnGoal = -(goalWidth * distanceBallToGoalLeft
	/ (distanceBallToGoalLeft + distanceBallToGoalRight)
	- theFieldDimensions.yPosLeftGoal);
    //one of the point of the standardShootLine obtain from intersection of angle bisector and groundLine
    Vector2f onGoal = Vector2f(theFieldDimensions.xPosOwnGroundline, yOnGoal);
    Vector2f lineDirection = Vector2f(ball.positionField.x() - onGoal.x(),
				      ball.positionField.y() - onGoal.y());
    Geometry::Line angleBisector = Geometry::Line(onGoal, lineDirection);
    if (Geometry::getIntersectionOfLines(
	angleBisector, defenderLine,
	intersectionOfAngleBisectorAndDefenderLine))
      return intersectionOfAngleBisectorAndDefenderLine;
    else
      return Vector2f(-3500.f, 0.f);
  }

  Vector2f
  LibSupporter::defenderPositionCoorperateWithKeeperAndSupporter (bool left)
  {
    Vector2f intersectionOfLineOfGoalAndBallAndDefenderLine;
    ball.getSuitablePosition();
    Vector2f goalLeft = Vector2f(theFieldDimensions.xPosOwnGroundline,
				 theFieldDimensions.yPosLeftGoal);
    Vector2f goalRight = Vector2f(theFieldDimensions.xPosOwnGroundline,
				  theFieldDimensions.yPosRightGoal);
    Vector2f Goal;
    //one of the point of the standardShootLine obtain from intersection of angle bisector and groundLine
    if (left)
    {
      Goal = goalLeft;
    }
    else
      Goal = goalRight;
    Vector2f lineDirection = Vector2f(ball.positionField.x() - Goal.x(),
				      ball.positionField.y() - Goal.y());
    Geometry::Line BallAndGoal = Geometry::Line(ball.positionField,
						lineDirection);
    if (Geometry::getIntersectionOfLines(
	BallAndGoal, defenderLine,
	intersectionOfLineOfGoalAndBallAndDefenderLine))
      return intersectionOfLineOfGoalAndBallAndDefenderLine;
    else
      return Vector2f(-3500.f, 0.f);
  }

  Vector2f
  LibSupporter::defenderPositionWhenKeeperNotPenalized ()
  {
    Vector2f defenderPosition;
//    Vector2f intersectionOfMainAngleBisectorAndDefenderLine = defenderPositionWhenKeeperPenalized();
//    Vector2f intersectionOfLeftLineAndDefenderLine;
//    Vector2f intersectionOfRightLineAndDefenderLine;
    Vector2f leftPoint;
    Vector2f rightPoint;
    Vector2f usingPoint;
    Vector2f goalLeft = Vector2f(theFieldDimensions.xPosOwnGroundline,
				 theFieldDimensions.yPosLeftGoal);
    Vector2f goalRight = Vector2f(theFieldDimensions.xPosOwnGroundline,
				  theFieldDimensions.yPosRightGoal);
    Vector2f leftLineDirection = Vector2f(
	ball.positionField.x() - theFieldDimensions.xPosOwnGroundline,
	ball.positionField.y() - theFieldDimensions.yPosLeftGoal);
    Vector2f rightLineDirection = Vector2f(
	ball.positionField.x() - theFieldDimensions.xPosOwnGroundline,
	ball.positionField.y() - theFieldDimensions.yPosRightGoal);
    Geometry::Line leftLine = Geometry::Line(goalLeft, leftLineDirection);
    Geometry::Line rightLine = Geometry::Line(goalRight, rightLineDirection);
    if (Geometry::getIntersectionOfLines(leftLine, defenderLine, leftPoint))
    {

    }
    if (Geometry::getIntersectionOfLines(rightLine, defenderLine, rightPoint))
    {

    }
    return leftPoint;
//    if(ballPosition.y() >= 0)
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
  LibSupporter::supporterPositionWhenKeeperNotPenalized ()
  {
    Vector2f defenderPosition;
//    Vector2f intersectionOfMainAngleBisectorAndDefenderLine = defenderPositionWhenKeeperPenalized();
//    Vector2f intersectionOfLeftLineAndDefenderLine;
//    Vector2f intersectionOfRightLineAndDefenderLine;
    Vector2f leftPoint;
    Vector2f rightPoint;
    Vector2f usingPoint;
    Vector2f goalLeft = Vector2f(theFieldDimensions.xPosOwnGroundline,
				 theFieldDimensions.yPosLeftGoal);
    Vector2f goalRight = Vector2f(theFieldDimensions.xPosOwnGroundline,
				  theFieldDimensions.yPosRightGoal);
    Vector2f leftLineDirection = Vector2f(
	ball.positionField.x() - theFieldDimensions.xPosOwnGroundline,
	ball.positionField.y() - theFieldDimensions.yPosLeftGoal);
    Vector2f rightLineDirection = Vector2f(
	ball.positionField.x() - theFieldDimensions.xPosOwnGroundline,
	ball.positionField.y() - theFieldDimensions.yPosRightGoal);
    Geometry::Line leftLine = Geometry::Line(goalLeft, leftLineDirection);
    Geometry::Line rightLine = Geometry::Line(goalRight, rightLineDirection);
    if (Geometry::getIntersectionOfLines(leftLine, defenderLine, leftPoint))
    {

    }
    if (Geometry::getIntersectionOfLines(rightLine, defenderLine, rightPoint))
    {

    }
    return leftPoint;
    /*
     if(ballPosition.y() >= 0)
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
  LibSupporter::defenderDefencePosition ()
  {
    Pose2f defencePosition;
//  float angle = ball.angleRad;
    float angle = ball.positionRobot.angle();
    angle = angle > (0.52f - robot.rotation) ? (0.52f - robot.rotation) : angle;
    angle =
	angle < (-0.52f - robot.rotation) ? (-0.52f - robot.rotation) : angle;
    bool leftGoal = false;

//  Vector2f ballSuitablePositionRelative = Geometry::fieldCoord2Relative(
//      Pose2f(robot.rotation, robot.x, robot.y),
//      Vector2f(ballPosition.x() - 150.f, ballPosition.y()));

//    Vector2f defenderPositionWhenKeeperPenalizedRelative = Geometry::fieldCoord2Relative(Pose2f(robot.rotation, robot.x, robot.y), defenderPositionWhenKeeperPenalized());
//  Vector2f defenderPositionWhenNotKeeperPenalizedRelative =
////        Geometry::fieldCoord2Relative(Pose2f(robot.rotation, robot.x, robot.y),
////                                      defenderPositionWhenKeeperNotPenalized());
//      Geometry::fieldCoord2Relative(
//          Pose2f(robot.rotation, robot.x, robot.y),
//          defenderPositionCoorperateWithKeeperAndSupporter(
//              leftGoal));
//    if(teammate.isKeeperPenalized == true)
//      defencePosition = Pose2f(angle, defenderPositionWhenKeeperPenalizedRelative.x(), defenderPositionWhenKeeperPenalizedRelative.y());
//    else
    Vector2f defenderPositionWhenNotKeeperPenalizedRelative =
	Geometry::fieldCoord2Relative(Pose2f(robot.rotation, robot.x, robot.y),
				      supporterStandPosition());
    defencePosition = Pose2f(
	angle, defenderPositionWhenNotKeeperPenalizedRelative.x(),
	defenderPositionWhenNotKeeperPenalizedRelative.y());
    return defencePosition;
  }

//for supporter
  Pose2f
  LibSupporter::supporterDefencePosition ()
  {
    Pose2f defencePosition;
    float angle = ball.angleRad;
    angle = angle > (0.52f - robot.rotation) ? (0.52f - robot.rotation) : angle;
    angle =
	angle < (-0.52f - robot.rotation) ? (-0.52f - robot.rotation) : angle;
    //Vector2f supporterPositionWhenKeeperPenalizedRelative = Geometry::fieldCoord2Relative(Pose2f(robot.rotation, robot(), robot.y), defenderPositionWhenKeeperPenalized());
    Vector2f supporterPositionWhenNotKeeperPenalizedRelative =
	Geometry::fieldCoord2Relative(
	    Pose2f(robot.rotation, robot.x, robot.y),
	    supporterPositionWhenKeeperNotPenalized());
//	  if(theTeammateData.isPenalized[1] == true)
//		  defencePosition = Pose2f(angle, defenderPositionWhenKeeperPenalizedRelative.x(), defenderPositionWhenKeeperPenalizedRelative.y();
//	  else
    defencePosition = Pose2f(
	angle, supporterPositionWhenNotKeeperPenalizedRelative.x(),
	supporterPositionWhenNotKeeperPenalizedRelative.y());
    return defencePosition;
  }

  Vector2f
  LibSupporter::globalSpeed (Vector2f relativeSpeed)
  {
    Vector2f speed;
//	  Vector2f startPointOfSpeedVectorGlobal = Geometry::relative2FieldCoord(Pose2f(robot.rotation, robot.x, robot.y), Vector2f(robot.x, robot.y) + relativeSpeed);
//	  Vector2f endPointOfSpeedVectorGlobal = Geometry::relative2FieldCoord(Pose2f(robot.rotation, robot.x, robot.y), relativeSpeed);
//	  speed = endPointOfSpeedVectorGlobal - startPointOfSpeedVectorGlobal;
    speed.x() = float(
	relativeSpeed.x() * cos(robot.rotation)
	    - relativeSpeed.y() * sin(robot.rotation));
    speed.y() = float(
	relativeSpeed.x() * sin(robot.rotation)
	    + relativeSpeed.y() * cos(robot.rotation));
    return speed;
  }

  float
  LibSupporter::yIfBallCanPassGroundLine ()
  {
//  ball.getSuitablePosition();
    Vector2f speed = globalSpeed(getBallSpeedRelative());
    if (speed.x() == 0 || speed.x() > -0.5f || ball.global.x() < robot.x)
    {
      return 5000.f;
    }
    float timeWhenAxisReached = (fabs(
	ball.positionField.x() - theFieldDimensions.xPosOwnGroundline)
	/ speed.x());

    Vector2f finalBallPosition = ball.positionField
	+ (speed * timeWhenAxisReached);
    return finalBallPosition.y();
  }

  bool
  LibSupporter::ifStopBallNeeded ()
  {
    if (ball.speedRelative.x() > 0 || ball.speedRelative.x() == 0)
    {
      return false;
    }
    else
    {
      if ((yIfBallCanPassGroundLine() > (theFieldDimensions.yPosLeftGoal + 100)
	  || yIfBallCanPassGroundLine()
	      < (theFieldDimensions.yPosRightGoal - 100))
	  || getBallEndPointRelative().x() < 300)
	return false;
      else
	return true;
    }
  }

  bool
  LibSupporter::toStopBall ()
  {

    if (ifStopBallNeeded() && robotAndBallCloseEnough())
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool
  LibSupporter::ifSpreadNeeded ()
  {
    if (!toStopBall())
    {
      return false;
    }
    else
    {
      if ((ball.yPosWhenBallReachesOwnYAxis() > 100
	  && ball.yPosWhenBallReachesOwnYAxis() < 600)
	  || (ball.yPosWhenBallReachesOwnYAxis() < -100
	      && ball.yPosWhenBallReachesOwnYAxis() > -600))
      {
	if (ball.timeWhenAxisReached < 2.f)
	  return true;
	else
	  return false;
      }

      else
	return false;
    }
  }

  bool
  LibSupporter::ifInterceptNeeded ()
  {
    if (!robotAndBallCloseEnough())
    {
      return false;
    }
    else
    {
      if ((ball.endPosition.y() > 100 && ball.endPosition.y() < 600)
	  || (ball.endPosition.y() < -100 && ball.endPosition.y() > -600))
      {
	if (ball.timeWhenAxisReached < 2.f && ball.timeWhenAxisReached > 0
	    && ball.endPosition.x() < 200.f)
	  return true;
	else
	  return false;
      }

      else
	return false;
    }
  }

  bool
  LibSupporter::robotAndBallCloseEnough ()
  {
    ball.getSuitablePosition();
    bool closeEnough = false;
    if ((ball.positionField.x() - robot.x) * (ball.positionField.x() - robot.x)
	+ (ball.positionField.y() - robot.y)
	    * (ball.positionField.y() - robot.y) < 1000 * 1000
	|| ball.positionRobot.x() * ball.positionRobot.x() + ball.positionRobot.y() * ball.positionRobot.y() < 1000 * 1000)
      closeEnough = true;
    else
      closeEnough = false;
    return closeEnough;
  }

  float
  LibSupporter::getYIfBallPassOwnAxis ()
  {
    Geometry::Line BallToRobot = Geometry::Line(ball.positionRobot,
						ball.speedRobot);
    Vector2f intersection;
    if (!Geometry::getIntersectionOfLines(
	BallToRobot, Geometry::Line(Vector2f(0, 0), Vector2f(0, 1)),
	intersection))
      return 5000.f;
    return intersection.y();
  }

	Pose2f LibSupporter::supporterPosition()
	{
		Vector2f StandPos(0.f, 0.f);
		Vector2f Goal;
		Vector2f intersection;
		StandPos.x() = -1000.f;
//		if(ball.teamPosition.x() > 1000.f)
//		{
//			StandPos.x() = -1000.f;
//		}
//		else
//		{
//			StandPos.x() = ball.teamPosition.x() - 2000.f;
//		}
//		if(StandPos.x() < -2800.f)
//		{
//			StandPos.x() = -2800.f;
//		}
		if (ball.teamPosition.y() < -500.f)
		{
			Goal = Vector2f(-4500.f, 400.f);
			lastIsLeft = true;
		}
		else if (ball.teamPosition.y() > 500.f)
		{
			Goal = Vector2f(-4500.f, -400.f);
			lastIsLeft = false;
		}
		else
		{
			Goal = lastIsLeft ? Vector2f(-4500.f, 400.f):Vector2f(-4500.f, -400.f);
		}
		Vector2f lineDirection = Vector2f(ball.teamPosition.x() - Goal.x(),
											ball.teamPosition.y() - Goal.y());
		Geometry::Line BallAndLeftGoal = Geometry::Line(ball.teamPosition,
								  lineDirection);
		if (Geometry::getIntersectionOfLines(
				BallAndLeftGoal,
			Geometry::Line(Vector2f(StandPos.x(), 0), Vector2f(0, 1)),
			intersection))
		{
			StandPos.y() = intersection.y();
		}
		else
		{
			StandPos.y() = 0.f;
		}
		StandPos = Geometry::fieldCoord2Relative(
					Pose2f(robot.rotation, robot.x, robot.y),
					StandPos);
		return Pose2f(ball.angleRad, StandPos);
	}

}

