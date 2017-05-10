/*
 *@Usage:Ball-searching method used by striker,supporter and stabber
 */
option(SearchForBall)//是否完整转一圈
{
  bool turnLeft = false;
  static unsigned int rotation_circle = 0;
  initial_state(lookAround)
  {
    transition
    {
//      if(state_time > 500 && ball.isTeamPositionValid)
//        goto lookAtTeamBall;
//      else if(state_time > 500)
        goto search;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRightDown;
    }
  }

  state(search)
  {
    transition
    {
    	if(action_done)
    	{
    		goto searchForBallTogether;
    	}
    }
    action
    {
//      theHeadControlMode = state_time % 6000 < 3000? HeadControl::lookLeftAndRightDown : HeadControl::lookLeftAndRightFast;
    	theHeadControlMode = state_time % 6000 < 3000? HeadControl::lookDown : HeadControl::lookForward;
      if(turnLeft)//??
      {
//        WalkAtSpeedPercentage(Pose2f(-0.7f, 0.f, 0.f));
    	  TurnOneRound(true);
      }
      else
      {
//        WalkAtSpeedPercentage(Pose2f(0.7f, 0.f, 0.f));
    	  TurnOneRound(false);
      }
    }
  }
  state(searchForBallTogether)
  {
	  transition
	  {

	  }
	  action
	  {
		  SearchForBallTogether();
	  }
  }
}
