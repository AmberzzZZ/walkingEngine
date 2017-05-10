#include "../LibraryBase.h"

namespace Behavior2015
{
#include "../Libraries/LibBall.h"
#include "../Libraries/LibArea.h"
#include "LibKeeper.h"
#include "LibDefender.h"

  LibBall::LibBall()
      : position(0.f, 0.f),
        distance(0.f),
        angleDeg(0.f),
        angleRad(0.f),
        timeWhenAxisReached(0.f),
        isTeamPositionValid(false)
  {
  }

  void LibBall::preProcess()
  {
    position = theBallModel.estimate.position;
    speedRelative = theBallModel.estimate.velocity;
    endPosition = BallPhysics::getEndPosition(position,speedRelative,theFieldDimensions.ballFriction);

    endPositionGlobal = Geometry::relative2FieldCoord(
        Pose2f(theRobotPose.rotation, theRobotPose.translation.x(),
               theRobotPose.translation.y()),
        endPosition);
    isTeamPositionValid = theTeamBallModel.isValid;

    if(isTeamPositionValid && ball.notSeenTime() > 1000.f)
	{
		positionField = teamPosition =theTeamBallModel.position;
		positionRobot = theRobotPose.inverse() * positionField;
		speedField = theTeamBallModel.velocity;
		speedRobot = Transformation::fieldToRobot (theRobotPose, speedField);
		endPositionField = theRobotPose * endPosition;
		endPositionRobot = endPosition;
		angleRad =  Transformation::fieldToRobot (theRobotPose,theTeamBallModel.position).angle();
		global = positionField;
		teamPosition = theTeamBallModel.position;
	}
	else
	{
		positionField = teamPosition =Transformation::robotToField(theRobotPose, position);
		positionRobot = position;
		speedRobot = speedRelative;
		speedField = Transformation::robotToField(theRobotPose, speedRobot);
		endPositionField = theRobotPose * endPosition;
		endPositionRobot = endPosition;
		angleRad = float(theBallModel.estimate.position.angle());

		global = Geometry::relative2FieldCoord(theRobotPose,ball.positionRobot);
		teamPosition = positionField;
	}
    distance = positionRobot.norm();
    angleDeg = float(toDegrees(angleRad));

    if(speedRelative.x() == 0 || x * speedRelative.x() > 0)
    {
      timeWhenAxisReached = 0.f;
    }
    else
    {
      timeWhenAxisReached = float(fabs(x / speedRelative.x())); 
    }

    if(notSeenTime() > 2000 && distance > 2000 && isTeamPositionValid)
    {
      position = defender.ballPositionRobot;
    }
  }

  void LibBall::postProcess()
  {
  }

  int LibBall::notSeenTime()
  {
    return theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  }

  float LibBall::yPosWhenBallReachesOwnYAxis()
  {
    if (speedRelative.x() == 0 || x * speedRelative.x() > 0)
    {
      return 0.0f;
    }
//    timeWhenAxisReached = float(fabs(x / speedRelative.x()));
    /**TODO it is a wrong calculation of y pisition */
    // one of the possible way to get y position, by the intersection of two lines
    Geometry::Line ballTrajectory = Geometry::Line(position,speedRelative);
    Geometry::Line ownAxis        = Geometry::Line(Vector2f(0,0),Vector2f(0,1));
    
    Vector2f intersection;
    
    if (Geometry::getIntersectionOfLines(ballTrajectory,ownAxis,intersection))
//    Vector2f finalBallPosition = theBallModel.estimate.position
//        + (speedRelative * timeWhenAxisReached);           //Original method
//    return finalBallPosition.y();
    return intersection.y();
  }

  bool LibBall::wasSeen()
  {
    return notSeenTime() < 500; //oroginally 300
  }

  bool LibBall::isInOwnPenalty()
  {
    if (ball.global.x() > -4500 && ball.global.x() < -3900
        && ball.global.y() < 1100 && ball.global.y() > -1100)
      return true;
    else
      return false;
  }

  bool LibBall::isInOppPenalty()
  {
    if (ball.global.x() < 4500 && ball.global.x() > 3900
        && ball.global.y() < 1100 && ball.global.y() > -1100)
      return true;
    else
      return false;
  }

  bool LibBall::isInOwnSide()
  {
    return x < -100;
  }

  bool LibBall::isInOppSide()
  {
    return x > 100;
  }

  bool LibBall::isNearMiddleLine()
  {
    return std::abs(x) < 100;
  }

  bool LibBall::isNearOppGoalLeft()
  {
    return area.nearOppGoalLeft(global.x(), global.y());
  }

  bool LibBall::isNearOppGoalRight()
  {
    return area.nearOppGoalRight(global.x(), global.y());
  }

  bool LibBall::isNearOppGoalMiddle()
  {
    return area.nearOppGoalMiddle(global.x(), global.y());
  }

  void LibBall::getSuitablePosition()
  {
    float scale = 1;	//the value representation the reliability of ball.global
    if (ball.wasSeen() || !ball.isTeamPositionValid)
    {
      scale = 1;
    }
    else if (ball.notSeenTime() > 1000)  //直觉上1000ms太短 @wzf
    {
      scale = 0;
    }
    else
    {
      scale = (1000 - ball.notSeenTime()) * 0.001f;
    }
    scale = scale > 1 ? 1 : scale;
    scale = scale < 0 ? 0 : scale;
    ball.suitable.x() = scale * ball.global.x()
        + (1 - scale) * ball.teamPosition.x();  //直觉上此处的线性化处理方式不合理 @wzf
    ball.suitable.y() = scale * ball.global.y()
        + (1 - scale) * ball.teamPosition.y();
  }

 void LibBall::getSuitablePosition2()
  {
//      if (!isTeamPositionValid)
//      {
//          suitable = global;
//      }
//      
//      if (theLocalizationTeamBall.isValid)
      
//      if (!ball.isTeamPositionValid)
//      {
//          ball.suitable = ball.global;
//      }
//      else
//      {
//          
//      }
//    float scale = 1;	    //the value representation the reliability of ball.global
//    if (!ball.isTeamPositionValid)
//    {
//      scale = 1;
//    }
//    else if (ball.notSeenTime() > 2000)  //直觉上1000ms太短 @wzf
//    {
//      scale = 0;
//    }
//    else
//    {
//      scale = (1000 - ball.notSeenTime()) * 0.001f;
//    }
//    scale = scale > 1 ? 1 : scale;
//    scale = scale < 0 ? 0 : scale;
//    ball.suitable.x() = scale * ball.global.x()
//        + (1 - scale) * ball.teamPosition.x();  //直觉上此处的线性化处理方式不合理 @wzf
//    ball.suitable.y() = scale * ball.global.y()
//        + (1 - scale) * ball.teamPosition.y();
  }

  Vector2f getBallAngleError(Vector2f globalPosition)
  {
//      Rotation
//      Geometry::Line robotToBall = Geometry::Line(Global,(Vector2f(robot.x,robor.y)-Vector2f(ball.x,ball.y)));
//      
//      Geometry::Line globalToBall = Geometry::Line(Global,(Vector2f(robot.x,robor.y)-Vector2f(ball.x,ball.y)));
      
  }
  
  bool LibBall::ifStopNeed()  //used only for keeper
  {
    bool ifStopNeed;
    if (ball.speedRelative.x() > 0 || ball.speedRelative.x() == 0)
    {
      ifStopNeed = false;
    }
    else
    {
      if (keeper.yIfBallCanPassGroundLine() > (theFieldDimensions.yPosLeftGoal + 100)
          || keeper.yIfBallCanPassGroundLine()
              < (theFieldDimensions.yPosRightGoal - 100))
        ifStopNeed = false;
      else
        ifStopNeed = true;
    }
    return ifStopNeed;
  }

}
