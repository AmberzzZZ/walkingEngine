option(GoToShootPoint2, (Vector2f)(Vector2f(4500.f, 0.f)) refPoint, (Vector2f)(Vector2f(80.f, 80.f)) tolerate)
{
  const float distance = 300.f;
  Vector2f shootPoint;
  shootPoint.x() = ball.global.x() + (ball.global.x()
      - refPoint.x()) * distance / (ball.global - refPoint).norm();
  shootPoint.y() = ball.global.y() + (ball.global.y()
      - refPoint.y()) * distance / (ball.global - refPoint).norm();
  Vector2f refPointRelative;
  refPointRelative = theRobotPose.inverse() * refPoint;
  Angle refAngle;
  refAngle = refPointRelative.angle();
  float distanceToLine = fabs((shootPoint.y() - robot.y) * ball.global.x() -
      (shootPoint.x() - robot.x) * ball.global.y() + shootPoint.x() * robot.y
      - shootPoint.y() * robot.x) / Vector2f(shootPoint.y() - robot.y, shootPoint.x() - robot.x).norm();

  if(theRole.role == Role::striker && ball.distance < 1000 && fabs(ball.angleRad) < 90_deg)
  {
    behavior.strikerOutput.isRequestNotAllowed = true;
    behavior.strikerOutput.isControlBall = true;
  }

  common_transition
  {

  }

  initial_state(judge)
  {
    transition
    {
      if(ball.distance < 500)
        goto goRoundBall;
      if(distanceToLine < 200 && refPointRelative.x() * ball.x < 0)
        goto walkToBall;
      else
        goto walkToShootPoint;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
    }
  }

  state(walkToBall)
  {
    transition
    {
      if(action_done || (ball.distance < 300.f && fabs(ball.angleRad) < 10_deg))
        goto goRoundBall;
      if((distanceToLine > 300.f || refPointRelative.x() * ball.positionRobot.x() > 0) && ball.distance > 500.f)
        goto walkToShootPoint;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToBall();
    }
  }

  state(walkToShootPoint)
  {
    transition
    {
      if(action_done || (ball.distance < 300 && fabs(ball.angleRad) > 45_deg))
        goto goRoundBall;
      if(distanceToLine < 200 && refPointRelative.x() * ball.positionRobot.x() < 0)
        goto walkToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToPoint(Pose2f(shootPoint.x(), shootPoint.y()),
                  Pose2f(8_deg, tolerate), false);
    }
  }

  state(goRoundBall)
  {
    transition
    {
      if(ball.distance > 500.f)
        goto judge;
      if(action_done)// || fabs(refAngle) < 5_deg)
        goto finish;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      GoBehindBall(refPoint.x(), refPoint.y());
    }
  }

  target_state(finish)
  {
    transition
    {
      if(ball.distance > 400 || fabs(ball.angleRad) > 10_deg || fabs(refAngle) > 10_deg)
        goto judge;
    }
    action
    {
      theHeadControlMode = (state_time % 4000) < 2000 ?
          HeadControl::lookLeftAndRight : HeadControl::lookAtBall;
      WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.f));
    }
  }

}
