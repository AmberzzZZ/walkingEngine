option(GoToBallAndKickDefender)
{
  bool useRightFoot;
  const float kickX = 220.f;
  const float kickY = 45.f;
  if(ball.positionRobot.y() < 0.f)
    useRightFoot = true;
  else
    useRightFoot = false;

  common_transition
  {

  }

  initial_state(start)
  {
    transition
    {
      if(action_done || (ball.positionRobot.norm() < 300 && fabs(goals.angleToGoal - ball.positionRobot.angle()) < 45_deg))
        goto alignToGoal;
    }
    action
    {
      GoToShootPoint();
    }
  }

  state(alignToGoal)
  {
    Pose2f turnSpeed(0.4f, 0.f, 0.f);
    transition
    {
      if(ball.positionRobot.norm() > 600 || fabs(ball.positionRobot.angle()) > 90_deg)
        goto start;
      else if(fabs(robot.angleToGoal) < 15_deg)
        goto alignBehindBall;
    }
    action
    {
      behavior.defenderOutput.isRequestNotAllowed = true;
      behavior.defenderOutput.isControlBall = true;
      theHeadControlMode = HeadControl::lookAtBall;
      if(robot.angleToGoal >= 0.f)
        WalkAtSpeedPercentage(turnSpeed);
      else
        WalkAtSpeedPercentage(-turnSpeed);
    }
  }

  state(alignBehindBall)
  {
    transition
    {
      if(ball.positionRobot.norm() > 600 || fabs(ball.positionRobot.angle()) > 90_deg)
        goto start;
      else if(fabs(robot.angleToGoal) > 20_deg)
        goto alignToGoal;
      else if(!useRightFoot && common.between(ball.positionRobot.y(), kickY -20.f, kickY + 20.f)
        && common.between(ball.positionRobot.x(), kickX -20.f, kickX + 20.f))
        goto kick; //using left foot
      else if(useRightFoot && common.between(ball.positionRobot.y(), -kickY -20.f, -kickY +20.f)
        && common.between(ball.positionRobot.x(), kickX -20.f, kickX + 20.f))
        goto kick; //using right foot
      else if(state_time > 3000)
          goto kick;
    }
    action
    {
      behavior.defenderOutput.isRequestNotAllowed = true;
      behavior.defenderOutput.isControlBall = true;
      theHeadControlMode = HeadControl::lookAtBall;
      if(ball.positionRobot.y() >= 0)
        WalkToTarget(Pose2f(0.4f, .7f, .7f),
                     Pose2f(0.f, ball.positionRobot.x() - kickX, ball.positionRobot.y() - kickY));
      if(ball.positionRobot.y()<0)
        WalkToTarget(Pose2f(0.4f, .7f, .7f),
                     Pose2f(0.f, ball.positionRobot.x() - kickX, ball.positionRobot.y() + kickY));
    }
  }

  state(kick)
  {
    transition
    {
      if(action_done || state_time > 1000)
        goto start;
    }
    action
    {
      behavior.defenderOutput.isRequestNotAllowed = true;
      behavior.defenderOutput.isControlBall = true;
      if(robot.pose.translation.x() < -1500)
        Kick(useRightFoot);
      else
        Kick(useRightFoot);
    }
  }

}
