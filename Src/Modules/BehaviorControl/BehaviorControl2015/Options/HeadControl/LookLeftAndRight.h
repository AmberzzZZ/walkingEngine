option (LookLeftAndRight, (float) (35_deg) panDeg, (float) (70_deg) speed)
{
  float panCenter = 0_deg;
  float tiltAngle = 0_deg;
  common_transition
  {

  }

  initial_state (lookLeft)
  {
    transition
    {
      if (action_done)
	goto lookRight;
    }
    action
    {
      SetHeadPanTilt (panCenter + panDeg, tiltAngle, speed);
    }
  }

  state (lookRight)
  {
    transition
    {
      if (action_done)
	goto lookLeft;
    }
    action
    {
      SetHeadPanTilt (panCenter - panDeg, tiltAngle, speed);
    }
  }
}
