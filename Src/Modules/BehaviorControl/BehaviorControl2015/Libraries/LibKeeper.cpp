/**
 * @file LibBall.cpp
 */

#include "../../BehaviorControl2015/LibraryBase.h"

namespace Behavior2015
{
#include "../../BehaviorControl2015/Libraries/LibBall.h"
#include "../../BehaviorControl2015/Libraries/LibKeeper.h"
#include "../../BehaviorControl2015/Libraries/LibRobot.h"

  LibKeeper::LibKeeper()
  {
  }

  void LibKeeper::preProcess()
  {
    positionOfKeeper = 0.0;
    ballPositionField = ball.positionField;//getBallPositionField();
    ballPositionRobot = ball.positionRobot;//getBallPositionRobot();
    ballSpeedField = ball.speedField;//getBallSpeedField();
    ballSpeedRobot = ball.speedRobot;//getBallSpeedRobot();
    ballEndPositionRobot = getBallEndPointRobot();
    ballEndPositionField = getBallEndPointField();
  }

  void LibKeeper::postProcess()
  {
  }

  /* to calculate the ball position when the ball come near using speed */
  float LibKeeper::yPosWhenBallReachesOwnYAxis()
  {
    if (ball.speedRelative.x() == 0 || ball.positionRobot.x() * ball.speedRelative.x() > 0)  //the ball isn't move or the ball is on the Opp_Field @lishu
    {
      return 0.0f;
    }
    float timeWhenAxisReached = float(fabs(ball.positionRobot.x() / ball.speedRelative.x()));
    Vector2f finalBallPosition = theBallModel.estimate.position
        + (ball.speedRelative * timeWhenAxisReached);
    return finalBallPosition.y();
  }
  float LibKeeper::yPosWhenBallReachesOwnYAxis2()
  {
    if (ballSpeedRobot.x() == 0 || ballPositionRobot.x() * ballSpeedRobot.x() > 0)  //the ball isn't move or the ball is on the Opp_Field @lishu
    {
      return 0.0f;
    }
    Geometry::Line BallToRobot = Geometry::Line(ballPositionRobot,
                                                      ballSpeedRobot);
        Vector2f intersection;
        if (!Geometry::getIntersectionOfLines(
        BallToRobot, Geometry::Line(Vector2f(0, 0), Vector2f(0, 1)),
        intersection))
            return 0.f;
    return intersection.y();
  }

  /* to calculate the ball position when the ball come near using position */
  float LibKeeper::toStandInTheWay()
  {
    float y_b = 0;
    std::vector<Vector2f>::const_iterator ballLastPosition = ballPath.end();
    // std::cout<<"globalx"<<global.x<<std::endl;
    if (initial_flag)
    {
      if (ball.wasSeen())
      {
        ballPath.push_back(theBallModel.estimate.position);
        initial_flag = false;
        //std::cout<<ball.x<<std::endl;
      }
    }
    else
    {
      if ((ball.positionRobot.x() - (ballLastPosition - 1)->x()) < -50)
      {
        ballPath.push_back(theBallModel.estimate.position);
        //std::cout<<ball.x<<std::endl;
      }
      else if ((ball.positionRobot.x() - (ballLastPosition - 2)->x()) > 0)
      {
        initial_flag = true;
        ballPath.clear();
      }
    }
    //std::cout<<ballPath.size()<<std::endl;
    if (ballPath.size() >= 8)
    {
      if (ball.positionRobot.x() > thePenaltyShootParameter.ballDangerous)
      {
        y_b = 0;
        for (int i = 1; i < 9; i++)
        {
          y_b += ((ballLastPosition - i)->x()
              * ((ballLastPosition - i - 4)->y())
              - (ballLastPosition - i)->y() * ((ballLastPosition - i - 4)->x()))
              / ((ballLastPosition - i)->x() - (ballLastPosition - i - 4)->x());
        }

      }
    }
    return y_b / 4;
  }

  float LibKeeper::toStandInTheWayGlobal()
  {
    float y_b = 0;
    std::vector<Vector2f>::const_iterator ballLastPosition = ballPath.end();
    ball.getSuitablePosition();
    if (initial_flag)
    {
      if (ball.wasSeen())
      {
        ballPath.push_back(ball.suitable);
        initial_flag = false;
      }
    }
    else
    {
      if ((ball.suitable.x() - (ballLastPosition - 1)->x()) < -50)
      {
        ballPath.push_back(ball.suitable);
      }
      else if ((ball.suitable.x() - (ballLastPosition - 2)->x()) > 0)
      {
        initial_flag = true;
        ballPath.clear();
      }
    }
    if (ballPath.size() >= 8)
    {
      y_b = 0;
      for (int i = 5; i < 9; i++)
      {
        y_b += (((ballLastPosition - i)->x() - theRobotPose.translation.x())
            * ((ballLastPosition - i - 4)->y())
            - (ballLastPosition - i)->y()
                * ((ballLastPosition - i - 4)->x()
                    - theRobotPose.translation.x()))
            / ((ballLastPosition - i)->x() - (ballLastPosition - i - 4)->x());
      }
      yBallPassRobot = y_b / 4;
    }
    else if (ballPath.size() >= 3)
    {
      yBallPassRobot = (((ballLastPosition - 1)->x()
          - theRobotPose.translation.x()) * ((ballLastPosition - 2)->y())
          - (ballLastPosition - 1)->y()
              * ((ballLastPosition - 2)->x() - theRobotPose.translation.x()))
          / ((ballLastPosition - 1)->x() - (ballLastPosition - 2)->x());
    }
    yBallPassRobot =
        (yBallPassRobot < 750 && yBallPassRobot > -750) ?
            yBallPassRobot : (yBallPassRobot > 750 ? 750 : -750);
    return yBallPassRobot;
  }

  /**@TODO Xiangwei Wang*/
  float LibKeeper::threshold(float yRobot)
  {
    return static_cast<float>(0.2
        - (yRobot / theFieldDimensions.yPosLeftGoal) * 0.1);
  }

  float LibKeeper::getPositionOfKeeper()
  {
    ball.getSuitablePosition();
    Vector2f goalLeft = Vector2f(theFieldDimensions.xPosOwnGroundline,
                                 theFieldDimensions.yPosLeftGoal);
    Vector2f goalRight = Vector2f(theFieldDimensions.xPosOwnGroundline,
                                  theFieldDimensions.yPosRightGoal);
    float goalWidth = theFieldDimensions.yPosLeftGoal
        - theFieldDimensions.yPosRightGoal;
    float distanceBallToGoalLeft = (goalLeft - ball.suitable).norm();
    float distanceBallToGoalRight = (goalRight - ball.suitable).norm();
    yOnGoal = -(goalWidth * distanceBallToGoalLeft
        / (distanceBallToGoalLeft + distanceBallToGoalRight)
        - theFieldDimensions.yPosLeftGoal);
    float yOnRobot = ball.suitable.y()
        + (yOnGoal - ball.suitable.y())
            * (ball.suitable.x() - theRobotPose.translation.x())
            / (ball.suitable.x() - theFieldDimensions.xPosOwnGroundline);
    yOnRobot =
        (yOnRobot > theFieldDimensions.yPosLeftGoal) ?
            theFieldDimensions.yPosLeftGoal : yOnRobot;
    yOnRobot =
        (yOnRobot < theFieldDimensions.yPosRightGoal) ?
            theFieldDimensions.yPosRightGoal : yOnRobot;
    float yRobot = robot.y > 0 ? robot.y : -robot.y;
    float hold = threshold(yRobot);
    if ((yOnRobot - positionOfKeeper) > hold
        || (yOnRobot - positionOfKeeper) < -hold)
    {
      positionOfKeeper = yOnRobot;
      return yOnRobot;
    }
    else
    {
      return positionOfKeeper;
    }
  }
float LibKeeper::getPositionOfKeeper2()
  {
    Vector2f goalLeft = Vector2f(theFieldDimensions.xPosOwnGroundline,
                                 theFieldDimensions.yPosLeftGoal);
    Vector2f goalRight = Vector2f(theFieldDimensions.xPosOwnGroundline,
                                  theFieldDimensions.yPosRightGoal);
    float goalWidth = theFieldDimensions.yPosLeftGoal
        - theFieldDimensions.yPosRightGoal;
    float distanceBallToGoalLeft = (goalLeft - ballPositionField).norm();
    float distanceBallToGoalRight = (goalRight - ballPositionField).norm();
    yOnGoal = -(goalWidth * distanceBallToGoalLeft
        / (distanceBallToGoalLeft + distanceBallToGoalRight)
        - theFieldDimensions.yPosLeftGoal);
    float yOnRobot = ballPositionField.y()
        + (yOnGoal - ballPositionField.y())
            * (ballPositionField.x() - theRobotPose.translation.x())
            / (ballPositionField.x() - theFieldDimensions.xPosOwnGroundline);
    yOnRobot =
        (yOnRobot > theFieldDimensions.yPosLeftGoal) ?
            theFieldDimensions.yPosLeftGoal : yOnRobot;
    yOnRobot =
        (yOnRobot < theFieldDimensions.yPosRightGoal) ?
            theFieldDimensions.yPosRightGoal : yOnRobot;
    float yRobot = robot.y > 0 ? robot.y : -robot.y;
    float hold = threshold(yRobot);
    if ((yOnRobot - positionOfKeeper) > hold
        || (yOnRobot - positionOfKeeper) < -hold)
    {
      positionOfKeeper = yOnRobot;
      return yOnRobot;
    }
    else
    {
      return positionOfKeeper;
    }
  }

  float LibKeeper::getPositionOfKeeper(float xRobot)
  {
    Vector2f goalLeft = Vector2f(theFieldDimensions.xPosOwnGroundline,
                                 theFieldDimensions.yPosLeftGoal);
    Vector2f goalRight = Vector2f(theFieldDimensions.xPosOwnGroundline,
                                  theFieldDimensions.yPosRightGoal);
    float goalWidth = theFieldDimensions.yPosLeftGoal
        - theFieldDimensions.yPosRightGoal;
    float distanceBallToGoalLeft = (goalLeft - ball.suitable).norm();
    float distanceBallToGoalRight = (goalRight - ball.suitable).norm();
    yOnGoal = -(goalWidth * distanceBallToGoalLeft
        / (distanceBallToGoalLeft + distanceBallToGoalRight)
        - theFieldDimensions.yPosLeftGoal);
    float yOnRobot = ball.suitable.y()
        + (yOnGoal - ball.suitable.y()) * (ball.suitable.x() - xRobot)
            / (ball.suitable.x() - theFieldDimensions.xPosOwnGroundline);
    yOnRobot =
        (yOnRobot > theFieldDimensions.yPosLeftGoal) ?
            theFieldDimensions.yPosLeftGoal : yOnRobot;
    yOnRobot =
        (yOnRobot < theFieldDimensions.yPosRightGoal) ?
            theFieldDimensions.yPosRightGoal : yOnRobot;
    float yRobot = robot.y > 0 ? robot.y : -robot.y;
    float hold = threshold(yRobot);
    if ((yOnRobot - positionOfKeeper) > hold
        || (yOnRobot - positionOfKeeper) < -hold)
    {
      positionOfKeeper = yOnRobot;
      return yOnRobot;
    }
    else
    {
      return positionOfKeeper;
    }
  }
float LibKeeper::getPositionOfKeeper2(float xRobot)
  {
    Vector2f goalLeft = Vector2f(theFieldDimensions.xPosOwnGroundline,
                                 theFieldDimensions.yPosLeftGoal);
    Vector2f goalRight = Vector2f(theFieldDimensions.xPosOwnGroundline,
                                  theFieldDimensions.yPosRightGoal);
    float goalWidth = theFieldDimensions.yPosLeftGoal
        - theFieldDimensions.yPosRightGoal;
    float distanceBallToGoalLeft = (goalLeft - ballPositionField).norm();
    float distanceBallToGoalRight = (goalRight - ballPositionField).norm();
    yOnGoal = -(goalWidth * distanceBallToGoalLeft
        / (distanceBallToGoalLeft + distanceBallToGoalRight)
        - theFieldDimensions.yPosLeftGoal);
    float yOnRobot = ballPositionField.y()
        + (yOnGoal - ballPositionField.y()) * (ballPositionField.x() - xRobot)
            / (ballPositionField.x() - theFieldDimensions.xPosOwnGroundline);
    yOnRobot =
        (yOnRobot > theFieldDimensions.yPosLeftGoal) ?
            theFieldDimensions.yPosLeftGoal : yOnRobot;
    yOnRobot =
        (yOnRobot < theFieldDimensions.yPosRightGoal) ?
            theFieldDimensions.yPosRightGoal : yOnRobot;
    float yRobot = robot.y > 0 ? robot.y : -robot.y;
    float hold = threshold(yRobot);
    if ((yOnRobot - positionOfKeeper) > hold
        || (yOnRobot - positionOfKeeper) < -hold)
    {
      positionOfKeeper = yOnRobot;
      return yOnRobot;
    }
    else
    {
      return positionOfKeeper;
    }
  }

  Pose2f LibKeeper::getPositionOfKeeper(float xRobot, bool flag)
  {
    ball.getSuitablePosition();
    Pose2f keeperPositionBallFarAway;  //@sxu 20140326
    if (ball.suitable.x() > 750.f)
    {
      Vector2f target = Geometry::fieldCoord2Relative(
          Pose2f(robot.rotation, robot.x, robot.y), Vector2f(-4250.f, 0.f));
      keeperPositionBallFarAway = Pose2f(-robot.rotation, target.x(),
                                         target.y());
      return keeperPositionBallFarAway;
    }
    else if (ball.suitable.x() < -750.f)
    {
      Pose2f PoseArc = getPositionOfKeeperOnArc();
      if (PoseArc.translation.x() < (xRobot - robot.x))	//PoseArc is a relative position
      {
        return PoseArc;
      }
      else
      {
        float yRobot = 0;
        Pose2f PoseKeeper;
        Vector2f target;
        if (flag)
        {
          yRobot = getPositionOfKeeper(xRobot);
          target = Geometry::fieldCoord2Relative(
              Pose2f(robot.rotation, robot.x, robot.y),
              Vector2f(xRobot, yRobot));
        }
        else
        {
          yRobot = getPositionOfKeeper();
          target = Geometry::fieldCoord2Relative(
              Pose2f(robot.rotation, robot.x, robot.y),
              Vector2f(robot.x, yRobot));

        }
        PoseKeeper = Pose2f(ball.angleRad, target.x(), target.y());
        return PoseKeeper;
      }
    }
    else
    {
      float keeperYPositionInTransitionArea;
      Pose2f keeperPositionInTransitionArea;
      keeperYPositionInTransitionArea = robot.y
          - robot.y * ((ball.suitable.x() + 750) / 1500);
      Vector2f target = Geometry::fieldCoord2Relative(
          Pose2f(robot.rotation, robot.x, robot.y),
          Vector2f(-4250.f, keeperYPositionInTransitionArea));
      keeperPositionInTransitionArea = Pose2f(-robot.rotation, target.x(),
                                              target.y());
      return keeperPositionInTransitionArea;
    }
  }
Pose2f LibKeeper::getPositionOfKeeper2(float xRobot, bool flag)
  {
    Pose2f keeperPositionBallFarAway;  //@sxu 20140326
    if (ballPositionField.x() > 750.f)
    {
      Vector2f target = Geometry::fieldCoord2Relative(
          Pose2f(robot.rotation, robot.x, robot.y), Vector2f(-4100.f, 0.f));
      keeperPositionBallFarAway = Pose2f(-robot.rotation, target.x(),
                                         target.y());
      return keeperPositionBallFarAway;
    }
    else if (ballPositionField.x() < -750.f)
    {
      Pose2f PoseArc = getPositionOfKeeperOnArc();
      if (PoseArc.translation.x() < (xRobot - robot.x))	//PoseArc is a relative position
      {
        return PoseArc;
      }
      else
      {
        float yRobot = 0;
        Pose2f PoseKeeper;
        Vector2f target;
        if (flag)
        {
          yRobot = getPositionOfKeeper2(xRobot);
          target = Geometry::fieldCoord2Relative(
              Pose2f(robot.rotation, robot.x, robot.y),
              Vector2f(xRobot, yRobot));
        }
        else
        {
          yRobot = getPositionOfKeeper2();
          target = Geometry::fieldCoord2Relative(
              Pose2f(robot.rotation, robot.x, robot.y),
              Vector2f(robot.x, yRobot));

        }
        PoseKeeper = Pose2f(ballPositionRobot.angle(), target.x(), target.y());
        return PoseKeeper;
      }
    }
    else
    {
      float keeperYPositionInTransitionArea;
      Pose2f keeperPositionInTransitionArea;
      keeperYPositionInTransitionArea = robot.y
          - robot.y * ((ballPositionField.x() + 750) / 1500);
      Vector2f target = Geometry::fieldCoord2Relative(
          Pose2f(robot.rotation, robot.x, robot.y),
          Vector2f(-4100.f, keeperYPositionInTransitionArea));
      keeperPositionInTransitionArea = Pose2f(-robot.rotation, target.x(),
                                              target.y());
      return keeperPositionInTransitionArea;
    }
  }

  /**@TODO sxu*/
  Pose2f LibKeeper::getPositionOfKeeperOnArc()
  {
    ball.getSuitablePosition();
    getPositionOfKeeper();  //to obtain the parameter yOnGoal
    Geometry::Line standardShootLine;  //the estimate line that the ball will go along
    Geometry::Circle goalKeeperCircle;	//the keeper should stand on the circle to save the ball(actually it is a semi-circle)
    Pose2f keeperPositionOnArc = Pose2f();
    /*two intersection of line and arc(or to say circle) */
    Vector2f firstPointOfLineAndArc = Vector2f(
        theFieldDimensions.xPosOwnGroundline, 0.f);
    Vector2f secondPointOfLineAndArc = Vector2f(
        theFieldDimensions.xPosOwnGroundline, 0.f);
    Vector2f onGoal = Vector2f(theFieldDimensions.xPosOwnGroundline, yOnGoal);  //one of the point of the standardShootLine obtain from intersection of angle bisector and groundLine
    Vector2f lineDirection = Vector2f(ball.suitable.x() - onGoal.x(),
                                      ball.suitable.y() - onGoal.y());
    standardShootLine = Geometry::Line(onGoal, lineDirection);
    Vector2f goalCenter = Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f);
    goalKeeperCircle = Geometry::Circle(goalCenter,
                                        theFieldDimensions.yPosLeftGoal);
    Geometry::getIntersectionOfLineAndCircle(standardShootLine,
                                             goalKeeperCircle,
                                             firstPointOfLineAndArc,
                                             secondPointOfLineAndArc);
    /*get the point that we need */
    if (firstPointOfLineAndArc.x() >= secondPointOfLineAndArc.x())
    {
      Vector2f target = Geometry::fieldCoord2Relative(
          Pose2f(robot.rotation, robot.x, robot.y), firstPointOfLineAndArc);
      keeperPositionOnArc = Pose2f(ball.angleRad, target.x(), target.y());
    }

    else
    {
      Vector2f target = Geometry::fieldCoord2Relative(
          Pose2f(robot.rotation, robot.x, robot.y), secondPointOfLineAndArc);
      keeperPositionOnArc = Pose2f(ball.angleRad, target.x(), target.y());
    }
    return keeperPositionOnArc;
  }  
  Pose2f LibKeeper::getPositionOfKeeperOnArc2()
  {
    getPositionOfKeeper2();  //to obtain the parameter yOnGoal
    Geometry::Line standardShootLine;  //the estimate line that the ball will go along
    Geometry::Circle goalKeeperCircle;	//the keeper should stand on the circle to save the ball(actually it is a semi-circle)
    Pose2f keeperPositionOnArc = Pose2f();
    /*two intersection of line and arc(or to say circle) */
    Vector2f firstPointOfLineAndArc = Vector2f(
        theFieldDimensions.xPosOwnGroundline, 0.f);
    Vector2f secondPointOfLineAndArc = Vector2f(
        theFieldDimensions.xPosOwnGroundline, 0.f);
    Vector2f onGoal = Vector2f(theFieldDimensions.xPosOwnGroundline, yOnGoal);  //one of the point of the standardShootLine obtain from intersection of angle bisector and groundLine
    Vector2f lineDirection = Vector2f(ballPositionField.x() - onGoal.x(),
                                      ballPositionField.y() - onGoal.y());
    standardShootLine = Geometry::Line(onGoal, lineDirection);
    Vector2f goalCenter = Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f);
    goalKeeperCircle = Geometry::Circle(goalCenter,
                                        theFieldDimensions.yPosLeftGoal);
    Geometry::getIntersectionOfLineAndCircle(standardShootLine,
                                             goalKeeperCircle,
                                             firstPointOfLineAndArc,
                                             secondPointOfLineAndArc);
    /*get the point that we need */
    if (firstPointOfLineAndArc.x() >= secondPointOfLineAndArc.x())
    {
      Vector2f target = Geometry::fieldCoord2Relative(
          Pose2f(robot.rotation, robot.x, robot.y), firstPointOfLineAndArc);
      keeperPositionOnArc = Pose2f(ballPositionRobot.angle(), target.x(), target.y());
    }

    else
    {
      Vector2f target = Geometry::fieldCoord2Relative(
          Pose2f(robot.rotation, robot.x, robot.y), secondPointOfLineAndArc);
      keeperPositionOnArc = Pose2f(ballPositionRobot.angle(), target.x(), target.y());
    }
    return keeperPositionOnArc;
  }

  /**@TODO sxu*/
  bool LibKeeper::keeperKickBallFarAway()
  {
    bool kickBall;
    bool inDangerousArea;
    float dangerousAreaRadiusSquare = theFieldDimensions.yPosLeftPenaltyArea
        * theFieldDimensions.yPosLeftPenaltyArea + 600 * 600;
//    float OUTdangerousAreaRadiusSquare = (theFieldDimensions.yPosLeftPenaltyArea+10)
//            * (theFieldDimensions.yPosLeftPenaltyArea+10) + 700 * 700;
//    globalBallSpeed = Geometry::relative2FieldCoord(Pose2f(robot.rotation, robot.x, robot.y), ball.speedRelative);

    if ((ball.positionField.x() - theFieldDimensions.xPosOwnGroundline)
        * (ball.positionField.x() - theFieldDimensions.xPosOwnGroundline)
        + ball.positionField.y() * ball.positionField.y() <= dangerousAreaRadiusSquare)
      inDangerousArea = true;
//    if (inDangerousArea && ((ball.suitable.x() - theFieldDimensions.xPosOwnGroundline)
//        * (ball.suitable.x() - theFieldDimensions.xPosOwnGroundline)
//        + ball.suitable.y() * ball.suitable.y() >= OUTdangerousAreaRadiusSquare))
    else
      inDangerousArea = false;
    if (inDangerousArea)
    {
      if (ballSpeedRobot.norm() < 20.f)	//(ball.speedRelative.x < 20 && ball.speedRelative.y < 20)
        kickBall = true;
      else
        kickBall = false;
    }
    else
      kickBall = false;
    return kickBall;
  }

  /**@TODO sxu*/
  Pose2f LibKeeper::keeperDefencePosition()
  {
    Pose2f keeperOnAngleBisector = getPositionOfKeeper(-4100, true);
    bool ballMoving;
    bool maybeGoal = ifSaveNeeded();
    Pose2f keeperOnMaybeGoalLine = Pose2f(-robot.rotation, 0.f,
                                          yPosWhenBallReachesOwnYAxis());
    Pose2f keeperShouldStand;
    float distanceBetweenBallAndRobotSquare = ball.positionRobot.x() * ball.positionRobot.x() + ball.positionRobot.y() * ball.positionRobot.y();
    if (ball.speedRelative.norm() < 20.f)
      ballMoving = false;
    else
      ballMoving = true;
    if (ballMoving)
    {
      if (maybeGoal)
        if (distanceBetweenBallAndRobotSquare > 2000 * 2000)
          keeperShouldStand = keeperOnAngleBisector;
        else if ((distanceBetweenBallAndRobotSquare <= (2000 * 2000))
            && (distanceBetweenBallAndRobotSquare > (1500 * 1500)))
        {
          float parameterYDirection = float(
              (sqrt(distanceBetweenBallAndRobotSquare) - 1500.f) / 500.f);
          float robotMoveInYDirectionRelative = parameterYDirection
              * keeperOnAngleBisector.translation.y()
              + (1 - parameterYDirection) * yPosWhenBallReachesOwnYAxis();
          Pose2f target = Pose2f(-robot.rotation, 0.f,
                                 robotMoveInYDirectionRelative);
          keeperShouldStand = target;
        }
        else
          keeperShouldStand = keeperOnMaybeGoalLine;
      else
        keeperShouldStand = keeperOnAngleBisector;
    }
    else
      keeperShouldStand = keeperOnAngleBisector;
    if (fabs(keeperShouldStand.rotation) > 0.52)
      keeperShouldStand.rotation =
          keeperShouldStand.rotation > 0 ? 0.52f : -0.52f;
    if (keeperShouldStand.translation.x() < -4500)
      keeperShouldStand.translation.x() = -4500;
    if (!ball.wasSeen() && ball.notSeenTime() > 3000
        && !ball.isTeamPositionValid)
    {
      keeperShouldStand.translation = Geometry::fieldCoord2Relative(
          Pose2f(robot.rotation, robot.x, robot.y), Vector2f(-4100, 0));
      keeperShouldStand.rotation = -robot.rotation;
    }
    return keeperShouldStand;
  }
  
Pose2f LibKeeper::keeperDefencePosition2()
  {
    Pose2f keeperOnAngleBisector = getPositionOfKeeper(-4250, true);
    bool ballMoving;
    bool maybeGoal = ifSaveNeeded2();
    Pose2f keeperOnMaybeGoalLine = Pose2f(-robot.rotation, 0.f,
                                          yPosWhenBallReachesOwnYAxis2());
    Pose2f keeperShouldStand;
    float distanceBetweenBallAndRobotSquare = ballPositionRobot.x() * ballPositionRobot.x()  + ballPositionRobot.y() * ballPositionRobot.y() ;
    if (ballSpeedRobot.norm() < 20.f)
      ballMoving = false;
    else
      ballMoving = true;
    if (ballMoving)
    {
      if (maybeGoal)
        if (distanceBetweenBallAndRobotSquare > 2000 * 2000)
          keeperShouldStand = keeperOnAngleBisector;
        else if ((distanceBetweenBallAndRobotSquare <= (2000 * 2000))
            && (distanceBetweenBallAndRobotSquare > (1500 * 1500)))
        {
          float parameterYDirection = float(
              (sqrt(distanceBetweenBallAndRobotSquare) - 1500.f) / 500.f);
          float robotMoveInYDirectionRelative = parameterYDirection
              * keeperOnAngleBisector.translation.y()
              + (1 - parameterYDirection) * yPosWhenBallReachesOwnYAxis2();
          Pose2f target = Pose2f(-robot.rotation, 0.f,
                                 robotMoveInYDirectionRelative);
          keeperShouldStand = target;
        }
        else
          keeperShouldStand = keeperOnMaybeGoalLine;
      else
        keeperShouldStand = keeperOnAngleBisector;
    }
    else
      keeperShouldStand = keeperOnAngleBisector;
    if (fabs(keeperShouldStand.rotation) > 0.52)
      keeperShouldStand.rotation =
          keeperShouldStand.rotation > 0 ? 0.52f : -0.52f;
    if (keeperShouldStand.translation.x() < -4500)
      keeperShouldStand.translation.x() = -4500;
    if (!ball.wasSeen() && ball.notSeenTime() > 3000
        && !ball.isTeamPositionValid && !useTeamBall)
    {
      keeperShouldStand.translation = Geometry::fieldCoord2Relative(
          Pose2f(robot.rotation, robot.x, robot.y), Vector2f(-4250, 0));
      keeperShouldStand.rotation = -robot.rotation;
    }
    return keeperShouldStand;
  }

  float LibKeeper::yIfBallCanPassGroundLine()
  {
    ball.getSuitablePosition();
    float x = ball.speedRelative.x() * (float) cos(robot.rotation)
        - ball.speedRelative.y() * (float) sin(robot.rotation);
    float y = ball.speedRelative.x() * (float) sin(robot.rotation)
        + ball.speedRelative.y() * (float) cos(robot.rotation);
    if (x == 0 || x > 0)
    {
      return 5000.f;
    }
    float timeWhenAxisReached = float(
        (fabs((ball.suitable.x() - theFieldDimensions.xPosOwnGroundline) / x)));
    if (timeWhenAxisReached>1.5f)
      return 5000.f;
    float finalY = ball.suitable.y() + timeWhenAxisReached * y;
    return finalY;
  }

  bool LibKeeper::ifSaveNeeded()
  {
    bool ifSaveNeed;
    if (ball.speedRelative.x() > 0 || ball.speedRelative.x() == 0)
    {
      ifSaveNeed = false;
    }
    else
    {
      if (yIfBallCanPassGroundLine() > (theFieldDimensions.yPosLeftGoal + 100)
          || yIfBallCanPassGroundLine()
              < (theFieldDimensions.yPosRightGoal - 100))
        ifSaveNeed = false;
      else
        ifSaveNeed = true;
    }
    return ifSaveNeed;
  }

  bool LibKeeper::ifSaveNeeded2()
  {
    bool ifSaveNeed;
    if (ballSpeedRobot.x() > 0 || ballSpeedRobot.x() == 0)
    {
      ifSaveNeed = false;
    }
    else
    {
      Geometry::Line GoalToRobot = Geometry::Line(ballPositionRobot,
                                                      ballSpeedRobot);
      Vector2f intersection;
//      Geometry::getIntersectionOfLines(
//              GoalToRobot, Geometry::Line(Vector2f(0, 0), Vector2f(0, 1)),
//              intersection);
//      std::cout<<"ballSpeedField--->"<<ballSpeedField.x()<<","<<ballSpeedField.y()<<std::endl;
    if (!Geometry::getIntersectionOfLines(
        GoalToRobot, Geometry::Line(Vector2f(0, 0), Vector2f(0, 1)),
        intersection))
        {
            return false;
        }
      if (intersection.y() > (theFieldDimensions.yPosLeftGoal)
          || intersection.y() < (theFieldDimensions.yPosRightGoal))
//          || (intersection.x()>150 || abs(ballEndPositionRobot.y())>800.f))
        ifSaveNeed = false;
      else
        ifSaveNeed = true;
    }
    return ifSaveNeed;
  }
  
  bool LibKeeper::ballDangerous()
  {
    if (ifSaveNeeded())
    {
      if (theBallModel.estimate.position.norm() < 1500)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  } 
  bool LibKeeper::ballDangerous2()
  {
//	  std::cout<<"ifSaveNeeded----->"<<ifSaveNeeded2()<<std::endl;
    if (ifSaveNeeded2())
    {
        float timeWhenBallReached = 1000;
        if (ball.positionRobot.x()>0)
        timeWhenBallReached = BallPhysics::timeForDistance(Vector2f(ball.speedRelative.x(),0),ball.positionRobot.x(),theFieldDimensions.ballFriction);
      if ((ball.positionRobot.norm() < 1500.f || (timeWhenBallReached < 1.0f && ball.positionRobot.norm() < 2000.f)))
//            && float(-ballPositionRobot.x())/float(ballSpeedRobot.x())<2.f)
//               &&ball.timeWhenAxisReached<2.5f
//               &&ball.timeWhenAxisReached>0)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }

  bool LibKeeper::ifDiveNeeded()
  {
    if (!ballDangerous())
    {
      return false;
    }
    else
    {
      if (ball.yPosWhenBallReachesOwnYAxis() > 600
          || ball.yPosWhenBallReachesOwnYAxis() < -600)
        return true;
      else
        return false;
    }
  }
  bool LibKeeper::ifDiveNeeded2()
  {
    if (!ballDangerous2())
    {
      return false;
    }
    else
    {
      if (yPosWhenBallReachesOwnYAxis2() > 500
          || yPosWhenBallReachesOwnYAxis2() < -500)
        return true;
      else
        return false;
    }
  }

  bool LibKeeper::turnAround()
  {
    bool turn = false;
    float maxDistance = 0;
    for (unsigned int i = 0; i != theFieldBoundary.boundaryOnField.size(); i++)
    {
      maxDistance =
          theFieldBoundary.boundaryOnField[i].x() > maxDistance ?
              theFieldBoundary.boundaryOnField[i].x() : maxDistance;
    }
    if (theJointAngles.angles[Joints::headYaw] < 0.087
        && theJointAngles.angles[Joints::headYaw] > -0.087)
    {
      if (maxDistance < 2500 && !ball.wasSeen())
      {
        turn = true;
      }
    }
    return turn;
  }

  bool LibKeeper::turnAroundOver()
  {
    bool turnOver = false;
    float maxDistance = 0;
    for (unsigned int i = 0; i != theFieldBoundary.boundaryOnField.size(); i++)
    {
      maxDistance =
          theFieldBoundary.boundaryOnField[i].x() > maxDistance ?
              theFieldBoundary.boundaryOnField[i].x() : maxDistance;
    }
    if (maxDistance > 6000.f)
    {
      turnOver = true;
    }
    return turnOver;
  }
  
   Vector2f LibKeeper::getBallPositionField()
   {
       Vector2f ownSeen = ball.global;
       if (ball.wasSeen() || ball.notSeenTime()<3000)
       {
           if (!theLocalizationTeamBall.isValid)
           {
               useTeamBall = false;
               return ownSeen;
           }
            else
            {
                Vector2f error = ownSeen - theLocalizationTeamBall.position;
                if (error.norm()<800.f)
                {
                    useTeamBall = false;
                    return ownSeen;
                }
                else
                {
                    if (theLocalizationTeamBall.numOfObservers>1 
                        && theFrameInfo.getTimeSince(theLocalizationTeamBall.lastObservation)<5000.f)
                        {
                            useTeamBall = true;
                            return theLocalizationTeamBall.position;
                        }
                    else 
                    {
                        useTeamBall = false;
                        return ownSeen;
                    }
                        
                }
            }
       }
       else 
       {
           if (!theLocalizationTeamBall.isValid)
           {
               useTeamBall = false;
               return ownSeen;
           }
            else
            {
                if (theLocalizationTeamBall.numOfObservers>0 
                    && theFrameInfo.getTimeSince(theLocalizationTeamBall.lastObservation)<8000.f)
                    {
                        useTeamBall = true;
                        return theLocalizationTeamBall.position;
                    }
                else 
                {
                    useTeamBall = false;
                    return ownSeen;
                }
                        
            }
       }
   }
   
  /**
   * @function get ball position to robot using both own observation and team info
   * @return ball poition to robot
   */
  Vector2f LibKeeper::getBallPositionRobot()
  {
      return Transformation::fieldToRobot(theRobotPose,ballPositionField);
  }

  /**
   * @function get ball speed on field
   * @return ball speed on field
   */
  Vector2f LibKeeper::getBallSpeedField()
  {
      if (useTeamBall)
          return theTeamBallModel.velocity;
      else 
          return Transformation::robotToField(theRobotPose,ball.speedRelative);
  }
  
  /**
   * @function get ball speed to robot
   * @return ball speed to robot
   */
  Vector2f LibKeeper::getBallSpeedRobot()
  {
      return Transformation::fieldToRobot(theRobotPose,ballSpeedField);
  }
  
  /**
   * @function get ball end point on field
   * @return ball end point on field
   */
  Vector2f LibKeeper::getBallEndPointField()
  {
      return Transformation::robotToField(theRobotPose,ballEndPositionRobot);
  }
  
  /**
   * @function get ball end point to robot
   * @return ball end point to robot
   */
   Vector2f LibKeeper::getBallEndPointRobot()
   {
       return BallPhysics::getEndPosition(ballPositionRobot,ballSpeedRobot,theFieldDimensions.ballFriction);
   }
   
   


}
