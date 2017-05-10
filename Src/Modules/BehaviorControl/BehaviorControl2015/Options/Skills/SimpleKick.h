/* kick action for remote control,
      kickType=0: kick forward
      kickType=1: kick left
      kickType=2: kick right */
option(SimpleKick, (int)(0) kickType)
{
  bool useRightFoot;
  common_transition
  {
    if(ball.notSeenTime() > 5000)
      goto searchForBall;
    if(ball.positionRobot.y() < 0.f || ball.positionRobot.y() == 0.f)
    {
      useRightFoot = true;
    }
    else if(ball.positionRobot.y() > 0.f)
    {
      useRightFoot = false;
    }
  }

  initial_state(turnToBall)
  {
    transition
    {
      if(ball.notSeenTime() > 7000)
        goto searchForBall;
      if(std::abs(ball.angleRad) < common.fromDegrees(5.))
        goto walkToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2f(.3f, .5f, .5f), Pose2f(ball.angleRad, 0.f, 0.f));
    }
  }

  state (walkToBall)
  {
    transition
    {
      if(ball.notSeenTime() > 7000)
        goto searchForBall;
//      if(kickType == 0 && common.between(ball.y, -50.f, 50.f)
//          && common.between(ball.x, 140.f, 170.f))
      if(kickType == 0 && (common.between(ball.positionRobot.y(), 30.f, 50.f) || common.between(ball.positionRobot.y(), -50.f, -30.f))
          && common.between(ball.positionRobot.x(), 170.f, 190.f))
        goto kick;
      if(kickType == 1 && common.between(ball.positionRobot.y(), -15.f, 15.f)
          && common.between(ball.positionRobot.x(), 140.f, 170.f))
        goto kickLeft;
      if(kickType == 2 && common.between(ball.positionRobot.y(), -15.f, 15.f)
          && common.between(ball.positionRobot.x(), 140.f, 170.f))
        goto kickRight;
      if(kickType == 3 && common.between(ball.positionRobot.y(), -50.f, 50.f)
          && common.between(ball.positionRobot.x(), 140.f, 170.f))
        goto kickFast;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      if(kickType == 0)
      {
//        if(ball.y >= 0)
        WalkToTarget(Pose2f(.1f, .5f, .5f),
                     Pose2f(0.f, ball.positionRobot.x() - 180.f, ball.positionRobot.y() - 40.f));
//        else
//        WalkToTarget(Pose2f(.1f, .5f, .5f),
//                     Pose2f(0.f, ball.x - 170.f, ball.y + 40.f));
      }
      if(kickType == 1)
        WalkToTarget(Pose2f(0.65f, 0.65f, 0.65f),
                     Pose2f(0, ball.positionRobot.x() - 150, ball.positionRobot.y()));
      if (kickType == 2)
        WalkToTarget(Pose2f(0.65f, 0.65f, 0.65f),
                     Pose2f(0, ball.positionRobot.x() - 150, ball.positionRobot.y()));
      if (kickType == 3)
        WalkToTarget(Pose2f(.1f, .5f, .5f),
                     Pose2f(0.f, ball.positionRobot.x() - 150.f, ball.positionRobot.y() - 30.f));
    }
  }

  state(kick)
  {
    transition
    {
      if(state_time > 4000 || (state_time > 500 && action_done))
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      if(useRightFoot)
      {
//        KickForward(KickRequest::kickForward, false);
        SpecialAction(SpecialActionRequest::kickBackHard);
      }
      else
      {
//        KickForward(KickRequest::kickForward, true);
        SpecialAction(SpecialActionRequest::kickBackHard,true);
      }
    }
  }

  state(kickLeft)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      InWalkKick(WalkRequest::sidewardsLeft,
                 Pose2f(0, ball.positionRobot.x() - 150.f, ball.positionRobot.y() - 20.f));
    }
  }

  state(kickRight)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      InWalkKick(WalkRequest::sidewardsRight,
                 Pose2f(0, ball.positionRobot.x() - 150.f, ball.positionRobot.y() + 20.f));
    }
  }

  state(kickFast)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      if(useRightFoot)
        InWalkKick(WalkRequest::right,
                   Pose2f(0, ball.positionRobot.x() - 150.f, ball.positionRobot.y() - 20.f));
      if(!useRightFoot)
        InWalkKick(WalkRequest::left,
                   Pose2f(0, ball.positionRobot.x() - 150.f, ball.positionRobot.y() + 20.f));
    }
  }

  state(searchForBall)
  {
    transition
    {
      if(ball.notSeenTime() < 300)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRight;
      WalkAtSpeedPercentage(Pose2f(1.f, 0.f, 0.f));
    }
  }
}
