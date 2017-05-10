option(Supporter2)
{
  common_transition
  {
    if(common.isKickOff)
      goto kickOff;

  }

  initial_state(start)
  {
    transition
    {
      goto play;
    }
  }

  state(kickOff)
  {
      if(state_time > 10000)
          common.isKickOff = false;
    transition
    {
      if(action_done || state_time > 20000)
        goto play;
    }
    action
    {
//      SupporterKickOff();

    	SupporterKickOffJudge();
    }
  }

  state(play)
  {
      transition
      {

      }
      action
      {
//          
    	  robot.isKickOffTypeConfirm = false;
          Supporter();
      }
  }

}
