option(KeeperLocation)
{
  initial_state(locating)
  {
    transition
    {
      if(keeper.turnAround())
        goto turnAround;
      else
        goto turnHead;
    }
    action
    {
      theHeadControlMode = HeadControl::lookScan;
      Stand();
      PlaySound("allright.wav");
    }
  }
  state(turnAround)
  {
    transition
    {
      if(keeper.turnAroundOver())
        goto compensateTime;
    }
    action
    {
      theHeadControlMode = HeadControl::lookScan;
      WalkAtSpeedPercentage(Pose2f(0.8f, 0.f, 0.f));
    }
  }
  state(turnHead)
  {
    transition
    {
      if(state_time > 6000)
      {
        if(ball.wasSeen())
          goto success;
        else
          goto lookForBall;
      }
    }
    action
    {
      KeeperHeadControl(0, 3000, 3000, 0);
      Stand();
      PlaySound("Nao.wav");
    }
  }
  state(compensateTime)
  {
    transition
    {
      if(state_time > 1000)
        goto turnHead;
    }
    action
    {
      WalkAtSpeedPercentage(Pose2f(0.8f, 0.f, 0.f));
    }
  }
  state(lookForBall)
  {
    transition
    {
      if(state_time < 5000)
      {
        if(ball.wasSeen())
          goto success;
      }
      else
      {
        goto success;
      }
    }
    action
    {
      KeeperHeadControl(0, 3000, 3000, 0);
      Stand();
    }
  }
  target_state(success)
  {

  }
}
