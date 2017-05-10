option(StrikerOld)
{
  static bool isKickOffLimit = false;
//  if(common.isKickOff)
//  {
//    isKickOffLimit = true;
//  }

//  behavior.strikerOutput.isKickOffLimit = isKickOffLimit;

  common_transition
  {
//    if(common.isKickOff)
//      goto kickOffBehavior;
    if(!ball.isTeamPositionValid && ball.notSeenTime() > 10000)
      goto searchForBallTogether;
  }

  initial_state(start)
  {
    transition
    {
      if(!ball.isTeamPositionValid && ball.notSeenTime() > 10000)
        goto searchForBallTogether;
      else
        goto goToBallAndKick;
    }
  }

  state(searchForBallTogether)
  {
    transition
    {
      if(action_done || ball.isTeamPositionValid || ball.notSeenTime() < 2000)
        goto start;
    }
    action
    {
      SearchForBallTogether();
    }
  }

  state(goToBallAndKick)
  {
    transition
    {
      if(action_done)
        goto start;
    }
    action
    {
      if(ball.global.x() > 4000)
        Dribble2(5000.f, 0.f);
      else
        GoToBallAndKick();
    }
  }

//  state(kickOffBehavior)
//  {
//    if(state_time > 10000)
//      common.isKickOff = false;
//    transition
//    {
//      if(action_done || !common.isKickOff)
//      {
//        isKickOffLimit = false;
//        goto start;
//      }
//    }
//    action
//    {
//      StrikerKickOff();
//    }
//  }

}
