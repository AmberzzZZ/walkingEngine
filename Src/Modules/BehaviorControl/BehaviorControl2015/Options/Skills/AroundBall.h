option(AroundBall, (Vector2f)(Vector2f(4500.f, 0.f)) target)
{
  const float ballDistance = 200.f;
  const float turnSpeed = 0.4f;
  const float leftSpeed = 0.7f;

  float angleToShoot = (theRobotPose.inverse() * target).angle();
  common_transition
  {

  }

  initial_state(start)
  {
    const Pose2f speed(0.5f, 0.f, 0.f);
    transition
    {
      if(fabs(ball.angleRad) < 15_deg)
      {
        if(common.between(ball.distance, ballDistance - 50.f, ballDistance + 50.f))
        {
          if(fabs(angleToShoot) > 30_deg)
            goto turnAround;
          else
            goto kickAlign;
        }
        else
        {
          goto closeToBall;
        }
      }
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      if(ball.angleRad > 0)
        WalkAtSpeedPercentage(speed);
      else
        WalkAtSpeedPercentage(-speed);
    }
  }

  state(closeToBall)
  {
    transition
    {
      if(fabs(ball.angleRad) < 25_deg)
      {
        if(common.between(ball.distance, ballDistance - 50.f, ballDistance + 50.f))
        {
          if(fabs(angleToShoot) > 30_deg)
            goto turnAround;
          else
            goto kickAlign;
        }
      }
      else
      {
        goto start;
      }
    }
    action
    {
      WalkToBall(ballDistance);
    }
  }

  state(turnAround)
  {
    transition
    {
//      if(ball.distance > 2 * ballDistance)
//        goto start;
      if(fabs(angleToShoot) < 5_deg)
        goto kickAlign;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      if(angleToShoot >= 0.f)
      {
        WalkAtSpeedPercentage(Pose2f(turnSpeed, 0.f, -leftSpeed));
      }
      else
      {
        WalkAtSpeedPercentage(Pose2f(-turnSpeed, 0.f, leftSpeed));
      }
    }
  }

  target_state(kickAlign)
  {
    transition
    {

    }
    action
    {
//      PlaySound("allright.wav");
      theHeadControlMode = HeadControl::lookAtBall;
      WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.f));
    }
  }

}
