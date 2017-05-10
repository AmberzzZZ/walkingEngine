option (HeadControl)
{
  common_transition
  {
    if (!theGroundContactState.contact && theGameInfo.state != STATE_INITIAL)
      goto lookForward;

    switch (theHeadControlMode)
    {
    case HeadControl::off:
      goto off;
    case HeadControl::lookForward:
      goto lookForward;
    case HeadControl::lookLeft:
      goto lookLeft;
    case HeadControl::lookRight:
      goto lookRight;
    case HeadControl::lookLeftAndRight:
      goto lookLeftAndRight;
    case HeadControl::lookLeftAndRightDown:
      goto lookLeftAndRightDown;
    case HeadControl::lookAtBall:
      goto lookAtBall;
    case HeadControl::lookAtTeamBall:
      goto lookAtTeamBall;
    case HeadControl::lookScan:
      goto lookScan;
    case HeadControl::lookLeftOrRightWhileDribble:
      goto lookLeftOrRightWhileDribble;
    case HeadControl::lookLeftAndRightFast:
      goto lookLeftAndRightFast;
    case HeadControl::lookAtOppKeeper:
      goto lookAtOppKeeper;
    case HeadControl::lookDown:
    	goto lookDown;
    default:
      goto none;
    }
  }

  initial_state (none)
  {
  }
  state (off)
  {
    action SetHeadPanTilt (JointAngles::off, JointAngles::off, 0.f);
  }
  state (lookForward)
  {
    action LookForward ();
  }
  state (lookLeft)
  {
    action LookLeft ();
  }
  state (lookRight)
  {
    action LookRight ();
  }
  state (lookLeftAndRight)
  {
    action LookLeftAndRight ();
  }
  state (lookLeftAndRightDown)
  {
    action LookLeftAndRightDown ();
  }
  state (lookAtBall)
  {
    action LookAtBall ();
  }
  state (lookAtTeamBall)
  {
    action LookAtTeamBall ();
  }
  state (lookScan)
  {
    action LookLeftAndRight (90_deg,80_deg);
  }
  state (lookLeftOrRightWhileDribble)
  {
    action LookLeftOrRightWhileDribble ();
  }
  state (lookLeftAndRightFast)
  {
    action LookLeftAndRightFast ();
  }
  state (lookAtOppKeeper)
  {
    action LookAtOppKeeper ();
  }
  state(lookDown)
  {
	  action LookForward(20_deg);
  }
}

struct HeadControl
{
  ENUM (Mode,
	{
  , none, off, lookForward, lookLeft, lookRight, lookLeftAndRight, lookLeftAndRightDown, lookAtBall, lookAtTeamBall,
     lookScan, lookLeftOrRightWhileDribble, lookLeftAndRightFast, lookAtOppKeeper, lookDown,});
};

HeadControl::Mode theHeadControlMode = HeadControl::Mode::none;	/**< The head control mode executed by the option HeadControl. */
