option(WalkToPoseBack, (Pose2f) pose, (Pose2f)(Pose2f(8_deg, 100.f, 100.f)) delta, (bool)(true) pause)
{
  const float farDistance = 400.f;
  Pose2f poseRobot = pose - robot.pose;
  Vector2f ballPositionRobot = ball.positionRobot;
  Vector2f ballPositionField = ball.global;

  initial_state(turnToBall)
  {
    Pose2f speedTurn(0.6f, 0.f, 0.f);
    Pose2f target(0.f, 0.f, 0.f);
    if(ball.notSeenTime() > 5000 && ball.isTeamPositionValid)
      target.rotation = (ballPositionField - robot.pose.translation).angle();
    else
      target.rotation = ballPositionRobot.angle();
    transition
    {
      if(common.between(robot.rotation, -5_deg, 5_deg))
        goto walkBack;
    }
    action
    {
      theHeadControlMode = state_time % 6000 < 3000? HeadControl::lookLeftAndRight:HeadControl::lookAtBall;
      WalkToTarget(speedTurn, target);
    }
  }

  state(walkBack)
  {
    Pose2f speedBack(0.f, 0.7f, 0.f);
    Pose2f target(0.f, poseRobot.translation.x(), 0.f);
    transition
    {
      if(!common.between(robot.rotation, -10_deg, 10_deg))
        goto turnToBall;
      if(common.between(poseRobot.translation.x(), -50.f, 50.f))
        goto walkSide;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRight;
      WalkToTarget(speedBack, target);
    }
  }

  state(walkSide)
  {
    Pose2f speedSide(0.f, 0.f, 0.7f);
    Pose2f target(0.f, 0.f, poseRobot.translation.y());
    transition
    {
      if(!common.between(robot.rotation, -10_deg, 10_deg))
        goto turnToBall;
      if(common.between(poseRobot.translation.y(), -50.f, 50.f))
        goto holdOn;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRight;
      WalkToTarget(speedSide, target);
    }
  }

  target_state(holdOn)
  {
    const Pose2f speed(0.3f, 0.3f, 0.3f);
    transition
    {

    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRight;
      if(pause)
        Stand();
      else
        WalkToTarget(speed, poseRobot);
    }
  }

};
