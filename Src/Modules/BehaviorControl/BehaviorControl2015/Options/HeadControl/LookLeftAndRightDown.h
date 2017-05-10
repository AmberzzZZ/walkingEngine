option (LookLeftAndRightDown, (float) (23_deg) panDeg, (float) (50_deg) speed)
{
  float panCenter = 0_deg;
  float tiltAngle = 20_deg;
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
