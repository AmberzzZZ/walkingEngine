option (GameFinished)
{
  initial_state (start)
  {
    transition
    {
      if (action_done)
	      goto playDead;
    }
    action
    {
      SpecialAction (SpecialActionRequest::sitDown);
    }
  }

  state (playDead)
  {
    action
    {
      SpecialAction (SpecialActionRequest::playDead);
    }
  }
}
