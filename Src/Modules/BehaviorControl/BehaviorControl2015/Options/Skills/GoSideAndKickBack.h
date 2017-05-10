/*
 * @File  GoSideAndKickBack.h
 * @Author dmx
 *
 * @Date April 18th.2014
 */

option(GoSideAndKickBack,(int) x,(int) y)
{
  float turnAngle;
  common_transition
  {
    if(fabs(ball.global.x() - x) < 200 && fabs(ball.global.y() - y) < 200)
      goto finish;
    else if(ball.distance > 600 && fabs(ball.angleDeg) > 6.f)
      goto turnToBallWhileWalkToBall;
  }

  initial_state(start)
  {
    transition
    {
      if (state_time > 400)
        goto walkToBall;
    }
    action
    {
      Stand();
    }
  }

  state(walkToBall)
  {
    transition
    {
      turnAngle = common.fromRad((theRobotPose.inverse() * Vector2f(x,y)).angle());
      if(ball.distance < 620.f &&
          common.between((float)fabs(turnAngle), 50.f, 110.f))
        goto walkStraightToBall;
      else if(ball.distance < 600.f && (turnAngle < -90 ||
          common.between(turnAngle, 0, 90)))
        goto goLeftsideOfBall;
      else if(ball.distance < 600.f && (turnAngle > 90 ||
          common.between(turnAngle, -90, 0)))
        goto goRightsideOfBall;
      else if(fabs(ball.angleDeg) > 5.f)
        goto turnToBallWhileWalkToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftOrRightWhileDribble;
      WalkToTarget(Pose2f(0.f, 0.7f, 0.f), ball.positionRobot);
    }
  }

  state(walkStraightToBall)
  {
    transition
    {
      if(ball.distance < 230)
        goto alignToTarget;
    }
    action
    {
      WalkToTarget(Pose2f(0.f, 0.7f, 0.f), ball.positionRobot);
    }
  }

  state(goLeftsideOfBall)
  {
    transition
    {
      if(ball.positionRobot.x()<200)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftOrRightWhileDribble;
      WalkToTarget(Pose2f(.6f, .7f, 0.f),
                   Pose2f(ball.angleRad + (float)atan2(300.f, ball.distance), ball.distance, 0));

    }
  }

  state(goRightsideOfBall)
  {
    transition
    {
      if(ball.positionRobot.x()<200)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftOrRightWhileDribble;
      WalkToTarget(Pose2f(.6f, .7f, 0.f),
                   Pose2f(ball.angleRad - (float)atan2(300.f, ball.distance), ball.distance, 0));

    }
  }

  state(turnToBall)
  {
    transition
    {
      if(fabs(ball.angleDeg) < 15)
        goto alignToTarget;
    }
    action
    {
      WalkToTarget(Pose2f(0.9f, 0.5f, 0.5f), Pose2f(ball.angleRad, 0.f, 0.f));
    }
  }

  state(alignToTarget)   //@XHan: Error happens here -->
  {
    transition
    {
      if(common.between(ball.positionRobot.y(), -8.f, 8.f)
          && common.between(ball.positionRobot.x(), 150.f, 170.f)  //@XHan:backup 170,190
          && (fabs((theRobotPose.inverse() * Vector2f(x,y)).angle()) - common.fromDegrees(80))
          < common.fromDegrees(8.f))
        goto kick;
      if (ball.distance > 800)
        goto start;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftOrRightWhileDribble;
      if((theRobotPose.inverse() * Vector2f(x,y)).angle() < 0)
        WalkToTarget(Pose2f(0.65f, 0.5f, 0.5f),
                     Pose2f((theRobotPose.inverse()*Vector2f(x,y)).angle()+common.fromDegrees(80),
                            ball.positionRobot.x() - 160.f, ball.positionRobot.y()));  //@XHan:backup 150
      else if((theRobotPose.inverse() * Vector2f(x,y)).angle() > 0)
        WalkToTarget(Pose2f(0.65f, 0.5f, 0.5f),
                     Pose2f((theRobotPose.inverse()*Vector2f(x,y)).angle()-common.fromDegrees(80),
                            ball.positionRobot.x() - 160.f, ball.positionRobot.y()));
    }
  }

  state(kick)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto finish;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      if((theRobotPose.inverse() * Vector2f(x,y)).angle() < 0)
        InWalkKick(WalkRequest::sidewardsLeft,
                   Pose2f(goals.angleToGoal, ball.positionRobot.x() - 160.f, ball.positionRobot.y() + 25));
      if((theRobotPose.inverse() * Vector2f(x,y)).angle() > 0)
        InWalkKick(WalkRequest::sidewardsRight,
                   Pose2f(goals.angleToGoal, ball.positionRobot.x() - 160.f, ball.positionRobot.y() - 25));
    }
  }

  state(turnToBallWhileWalkToBall)
  {
    transition
    {
      if (fabs(ball.angleDeg) < 5.f && state_time > 500)
        goto walkToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(ball.angleRad, 0.f, 0.f));
    }
  }

  target_state(finish)
  {
    action
    {
      theHeadControlMode=HeadControl::lookLeftAndRight;
      Stand();
    }
  }

}
