/**
 * @file FlashKick.h
 * @author Chenzd
 * @date Feb 24, 2017
 * @Usage kick ball fast but not accurate and distant
 * @param useRightFoot: as the name says; power: [0,1], decide the kick power; kickDirection: [-60_deg, 60_deg] is OK.
 */

option(FlashKick, (bool)(true) useRightFoot, (float)(1.f) power, (Angle)(0.f) kickDirection)
{
  initial_state(start)
  {
    transition
    {
      if(state_time>1000)
        goto judge;
    }
    action
    {
      WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.f));
    }
  }
  state(judge)
  {
    transition
    {
      if(useRightFoot)
        goto kickRight;
      else
        goto kickLeft;
    }
    action
    {
    }
  }

  state(kickLeft)
  {
    transition
    {
      if(theWalkingEngineOutput.kickFinish && (state_time > 100))
        goto step;
    }
    action
    {
      LookAtBall();
      FastKick(WalkRequest::left, Pose2f(kickDirection, power, 0.f), true);
    }
  }

  state(kickRight)
  {
    transition
    {
      if(theWalkingEngineOutput.kickFinish && (state_time > 100))
        goto step;
    }
    action
    {
      LookAtBall();
      FastKick(WalkRequest::right, Pose2f(kickDirection, power, 0.f), true);
    }
  }
  state(step)
  {
    transition
    {
      if(state_time>1000)
        goto finish;
    }
    action
    {
      WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.f));
    }
  }
  target_state(finish)
  {
    transition
    {
    }
    action
    {
    }
  }

}
