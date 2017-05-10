/**
 * @file Kick.h
 * @author yongqi Lee
 * @date Jul 7, 2016
 * @Usage kick ball using run swift's kick
 * @param useRightFoot: as the name says; power: [0,1], decide the kick power; kickDirection: [-60_deg, 60_deg] is OK.
 */

option(Kick, (bool)(true) useRightFoot, (float)(1.f) power, (Angle)(0.f) kickDirection)
{
  initial_state(judge)
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
      if(theWalkingEngineOutput.kickFinish && (state_time > 1000))
        goto finish;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      FastKick(WalkRequest::left, Pose2f(kickDirection, power, 0.f));
    }
  }

  state(kickRight)
  {
    transition
    {
      if(theWalkingEngineOutput.kickFinish && (state_time > 1000))
        goto finish;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      FastKick(WalkRequest::right, Pose2f(kickDirection, power, 0.f));
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
