/**
 * @file LibPlayer.cpp
 */

#include "../../BehaviorControl2015/LibraryBase.h"

namespace Behavior2015
{
#include "../../BehaviorControl2015/Libraries/LibPlayer.h"
#include "../../BehaviorControl2015/Libraries/LibRobot.h"
#include "../../BehaviorControl2015/Libraries/LibBall.h"
#include "../../BehaviorControl2015/Libraries/LibTeammate.h"

  LibPlayer::LibPlayer()
      : yKeeper(1100),
        lastSeenPosition(1.f),
        dynamicRoleAssign(false),
        strikerKickOff(0),
        playTime(0)
  {
  }

  void LibPlayer::preProcess()
  {
    angleInPenaltyLeft = (theRobotPose.inverse()
        * Vector2f(
            theFieldDimensions.xPosOpponentGoalPost,
            theFieldDimensions.yPosLeftGoal
                + thePenaltyShootParameter.offsetLeft)).angle();
    angleInPenaltyRight = (theRobotPose.inverse()
        * Vector2f(
            theFieldDimensions.xPosOpponentGoalPost,
            theFieldDimensions.yPosRightGoal
                + thePenaltyShootParameter.offsetRight)).angle();

    if (random == 0)
      angleInPenalty = angleInPenaltyLeft;
    else
      angleInPenalty = angleInPenaltyRight;
    angleInKickBackward = (theRobotPose.inverse()
        * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle();

    if (yKeeper > 0 && std::abs(yKeeper) < 1100)
      yOffset = (yKeeper - 1100) / 2;
    else if (yKeeper <= 0 && std::abs(yKeeper) < 1100)
      yOffset = (yKeeper + 1100) / 2;

    if (ball.notSeenTime() > 2000 && ball.isTeamPositionValid)
    {
      xBall = ball.teamPosition.x();
      yBall = ball.teamPosition.y();
    }
    else
    {
      xBall = ball.global.x();
      yBall = ball.global.y();
    }

    if (ball.notSeenTime() > 2000 && ball.isTeamPositionValid)
    {
      ballAngleRad = theRobotPose.rotation
          - (Vector2f(ball.teamPosition.x(), ball.teamPosition.y())
              - Vector2f(theRobotPose.translation.x(),
                         theRobotPose.translation.y())).angle();
    }
    else
    {
      ballAngleRad = ball.angleRad;
    }

    LOM = 11 / 6 * (xBall + 4500);
    MOR = -11 / 6 * (xBall + 4500);
    defendLine = -2500;
    kickLine = 700;
    target = Geometry::fieldCoord2Relative(
        Pose2f(theRobotPose.rotation, theRobotPose.translation.x(),
               theRobotPose.translation.y()),
        Vector2f(
            -2500,
            (-2500 * (yBall - yOffset) + (xBall * yOffset + 4500 * yBall))
                / (4500 + xBall)));
    targetMid = Geometry::fieldCoord2Relative(
        Pose2f(theRobotPose.rotation, theRobotPose.translation.x(),
               theRobotPose.translation.y()),
        Vector2f(-3500, 1000 / (4500 + xBall) * yBall));
    targetLeft = Geometry::fieldCoord2Relative(
        Pose2f(theRobotPose.rotation, theRobotPose.translation.x(),
               theRobotPose.translation.y()),
        Vector2f(-4200, 1100));
    targetLeft = Geometry::fieldCoord2Relative(
        Pose2f(theRobotPose.rotation, theRobotPose.translation.x(),
               theRobotPose.translation.y()),
        Vector2f(-4200, -1100));
    defendPosition = Geometry::fieldCoord2Relative(
        Pose2f(theRobotPose.rotation, theRobotPose.translation.x(),
               theRobotPose.translation.y()),
        Vector2f(-2500, 0));

    // dynamicRoleAssign = true;
    //@han:kick-off flag
    if (theGameInfo.state == STATE_SET)
      playTime = 0;
    if (theGameInfo.state == STATE_PLAYING && playTime == 0)	//checked
      playTime = theFrameInfo.time;
    if (strikerKickOff && theGameInfo.state == STATE_PLAYING)
    {
      if (theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
      {
        if (ball.global.norm() > 900)  //850
        {
          strikerKickOff = 0;
        }
      }
      else
      {
        if (ball.speedRelative.norm() > 5
            || (playTime != 0 && theFrameInfo.getTimeSince(playTime) > 10000)
            || ball.global.norm() > 900)
        {
          strikerKickOff = 0;
        }
      }
      if (ball.isTeamPositionValid && ball.teamPosition.norm() > 900)
        strikerKickOff = 0;
      if ((playTime != 0 && theFrameInfo.getTimeSince(playTime) > 15000))
        strikerKickOff = 0;
    }    
  }

  void LibPlayer::postProcess()
  {
  }

}
