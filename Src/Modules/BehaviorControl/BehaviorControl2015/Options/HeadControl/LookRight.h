option (LookRight, (float) (60_deg) panDeg, (float) (70_deg) speed, (int)(2000) rightTime, (int)(2000) forwardTime)
{
  float panCenter = 0_deg;
  float tiltAngle = 0_deg;
  common_transition
  {

  }

  initial_state (lookRight)
  {
    transition
    {
      if (action_done)
        goto rightWait;
    }
    action
    {
      SetHeadPanTilt (panCenter - panDeg, tiltAngle, speed);
    }
  }

  state (rightWait)
  {
    transition
    {
      if (state_time > rightTime)
        goto lookForward;
    }
    action
    {
      SetHeadPanTilt (panCenter - panDeg, tiltAngle, speed);
    }
  }

  state (lookForward)
  {
    transition
    {
      if (action_done)
        goto forwardWait;
    }
    action
    {
      SetHeadPanTilt (panCenter, tiltAngle, speed);
    }
  }

  state (forwardWait)
  {
    transition
    {
      if (state_time > forwardTime)
        goto lookRight;
    }
    action
    {
      SetHeadPanTilt (panCenter, tiltAngle, speed);
    }
  }
}
