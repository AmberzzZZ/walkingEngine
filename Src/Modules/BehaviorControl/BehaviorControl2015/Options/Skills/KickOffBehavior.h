option(KickOffBehavior)
{
  bool useRightFoot;
  const Pose2f ReadyPosestrikerPose = Pose2f{ 0, -1000, 0};

  common_transition
  {
    if(!player.strikerKickOff)
      goto normalPlay;
    if(ball.positionRobot.y() < 0.f || ball.positionRobot.y() == 0.f)
    {
      useRightFoot = true;
    }
    else if(ball.positionRobot.y() > 0.f)
    {
      useRightFoot = false;
    }
  }

  initial_state(play)
  {
    transition
    {
      if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
        goto goToBall;
      else
        goto notKickOff;
    }
    action
    {
      Stand();
    }
  }
  //kick-off situation
  state(goToBall)
  {
    transition
    {
      if(Vector2f(ball.positionRobot.x(), ball.positionRobot.y()).norm() < 1000)
        goto kickOff;
      else if(action_done)
        goto normalPlay;
    }
    action
    {
      WalkToBall();
    }
  }
  state(kickOff)
  {
    transition
    {
      if(Vector2f(ball.positionRobot.x(), ball.positionRobot.y()).norm() > 1200)
        goto goToBall;
      else if(ball.global.norm() > 1500 || action_done)
        goto normalPlay;
    }
    action
    {
      Dribble(2000.f, -1500.f);
    }
  }

  //not kick-off team
  state(notKickOff)
  {
    transition
    {
      if(player.strikerKickOff == 0)
        goto normalPlay;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRight;
      if((Vector2f(robot.x, robot.y) - ReadyPosestrikerPose.translation).norm() > 3000)
      WalkToPose(ReadyPosestrikerPose, Pose2f(3_deg, 20.f, 20.f), 1);
    }
  }

  target_state(normalPlay)
  {
    transition
    {

    }
    action
    {
      player.strikerKickOff = 0;
      theHeadControlMode = HeadControl::lookLeftAndRight;
      Stand();
    }
  }
}
