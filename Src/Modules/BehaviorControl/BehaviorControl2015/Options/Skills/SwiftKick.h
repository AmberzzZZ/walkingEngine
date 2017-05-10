option(SwiftKick)
{
  const float kickX = 170.f;
  const float kickY = 50.f;
  float angleToShoot = goals.angleToShoot();
  common_transition
  {
    if(ball.notSeenTime() > 7000)
      goto searchForBall;
  }

  initial_state(start)
  {
    transition
    {
      if(ball.notSeenTime() > 5000)
        goto searchForBall;
      else if(ball.distance > 600)
        goto walkToBall;
      else
        goto kickAlign;
    }
    action
    {

    }
  }

  state(searchForBall)
  {
    transition
    {
      if(ball.notSeenTime() < 1000)
        goto start;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRight;
      Pose2f speed(.5f, 0.f, 0.f);
      WalkAtSpeedPercentage(speed);
    }
  }

  state(walkToBall)
  {
    transition
    {
      if(ball.distance < 400)
      {
        if(fabs(ball.angleRad) > 15_deg)
          goto turnToBall;
        else if(fabs(angleToShoot) > 60_deg)
          goto aroundBall;
        else
          goto kickAlign;
      }
    }
    action
    {
      WalkToBall();
    }
  }

  state(turnToBall)
  {
    transition
    {
      if(fabs(ball.angleRad) > 7_deg)
        goto walkToBall;
    }
    action
    {
      TurnToBall();
    }
  }

  state(aroundBall)
  {
    transition
    {
      if(action_done || fabs(angleToShoot) < 30_deg)
        goto walkToBall;
    }
    action
    {
      AroundBall();
    }
  }

  state(kickAlign)
  {
    transition
    {
      if(ball.distance > 600)
        goto walkToBall;
      else if(ball.notSeenTime() > 1000)
        goto searchForBall;
      else if(fabs(angleToShoot) < 7_deg && common.between(ball.x - kickX, -10.f, 10.f) && common.between(ball.positionRobot.y() - kickY, -10.f, 10.f))
        goto kickBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      Pose2f speed(.3f, .3f, .3f);
      WalkToTarget(speed, Pose2f(angleToShoot, ball.positionRobot.x() - kickX, ball.positionRobot.y() - kickY));
      behavior.striker.isKickOffLimit = true;
      behavior.striker.kickOffDirection = StrikerBehavior::LEFT;
      behavior.striker.isControlBall = true;
    }
  }

  state(kickBall)
  {
    transition
    {
      if(state_time > 2000)
        goto start;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      InWalkKick(WalkRequest::left,Pose2f(0,0,0));
      behavior.striker.requestRoleType = Role::supporter;
    }
  }

}
