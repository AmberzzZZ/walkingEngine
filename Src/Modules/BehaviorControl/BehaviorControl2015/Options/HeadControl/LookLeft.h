option (LookLeft, (float) (60_deg) panDeg, (float) (70_deg) speed, (int)(2000) leftTime, (int)(2000) forwardTime)
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
        goto leftWait;
    }
    action
    {
      SetHeadPanTilt (panCenter + panDeg, tiltAngle, speed);
    }
  }

  state (leftWait)
  {
    transition
    {
      if (state_time > leftTime)
        goto lookForward;
    }
    action
    {
      SetHeadPanTilt (panCenter + panDeg, tiltAngle, speed);
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
        goto lookLeft;
    }
    action
    {
      SetHeadPanTilt (panCenter, tiltAngle, speed);
    }
  }
}
