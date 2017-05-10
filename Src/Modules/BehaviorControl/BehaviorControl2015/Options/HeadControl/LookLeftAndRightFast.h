/*
 * @Author: Chenzd
 * @Usage : Fast turn head like UT
 * @Date  : March 10 2017
 */

option (LookLeftAndRightFast, (float) (60_deg) panDeg, (float) (150_deg) speed)
{
  float panCenter = 0_deg;
  float tiltAngle = 10_deg;
  float tiltAngleBound = 0_deg;
  float panDegAvg = 30_deg;
  float waitTime = 300;
  common_transition
  {

  }

  initial_state (first)
  {
    transition
    {
      if (action_done)
    	  goto waitFirst;
    }
    action
    {
      SetHeadPanTilt (panCenter + 1 * panDegAvg, tiltAngle, speed);
    }
  }
  state (waitFirst)
  {
    transition
    {
      if (state_time > waitTime)
    	  goto second;
    }
    action
    {
      SetHeadPanTilt (panCenter + 1 * panDegAvg, tiltAngle, speed);
    }
  }

  state (second)
  {
    transition
    {
      if (action_done)
    	  goto waitSecond;
    }
    action
    {
      SetHeadPanTilt (panCenter + 2 * panDegAvg, tiltAngle, speed);
    }
  }

  state (waitSecond)
  {
    transition
    {
      if (state_time > waitTime)
    	  goto third;
    }
    action
    {
      SetHeadPanTilt (panCenter + 2 * panDegAvg, tiltAngle, speed);
    }
  }

  state (third)
  {
    transition
    {
      if (action_done)
    	  goto waitThird;
    }
    action
    {
      SetHeadPanTilt (panCenter + 1 * panDegAvg, tiltAngle, speed);
    }
  }
  state (waitThird)
  {
    transition
    {
      if (state_time > waitTime)
    	  goto fourth;
    }
    action
    {
      SetHeadPanTilt (panCenter + 1 * panDegAvg, tiltAngle, speed);
    }
  }

  state (fourth)
  {
    transition
    {
      if (action_done)
    	  goto waitFourth;
    }
    action
    {
      SetHeadPanTilt (panCenter + 0 * panDegAvg, tiltAngle, speed);
    }
  }
  state (waitFourth)
  {
    transition
    {
      if (state_time > waitTime)
    	  goto fifth;
    }
    action
    {
      SetHeadPanTilt (panCenter + 0 * panDegAvg, tiltAngle, speed);
    }
  }

  state (fifth)
  {
    transition
    {
      if (action_done)
    	  goto waitFifth;
    }
    action
    {
      SetHeadPanTilt (panCenter - 1 * panDegAvg, tiltAngle, speed);
    }
  }
  state (waitFifth)
  {
    transition
    {
      if (state_time > waitTime)
    	  goto sixth;
    }
    action
    {
      SetHeadPanTilt (panCenter - 1 * panDegAvg, tiltAngle, speed);
    }
  }

  state (sixth)
  {
    transition
    {
      if (action_done)
    	  goto waitSixth;
    }
    action
    {
      SetHeadPanTilt (panCenter - 2 * panDegAvg, tiltAngle, speed);
    }
  }
  state (waitSixth)
  {
    transition
    {
      if (state_time > waitTime)
    	  goto seventh;
    }
    action
    {
      SetHeadPanTilt (panCenter - 2 * panDegAvg, tiltAngle, speed);
    }
  }

  state (seventh)
  {
    transition
    {
      if (action_done)
    	  goto waitSeventh;
    }
    action
    {
      SetHeadPanTilt (panCenter - 1 * panDegAvg, tiltAngle, speed);
    }
  }
  state (waitSeventh)
  {
    transition
    {
      if (state_time > waitTime)
    	  goto eighth;
    }
    action
    {
      SetHeadPanTilt (panCenter - 1 * panDegAvg, tiltAngle, speed);
    }
  }

  state (eighth)
  {
    transition
    {
      if (action_done)
    	  goto waitEigth;
    }
    action
    {
      SetHeadPanTilt (panCenter - 0 * panDegAvg, tiltAngle, speed);
    }
  }
  state (waitEigth)
  {
    transition
    {
      if (state_time > waitTime)
    	  goto first;
    }
    action
    {
      SetHeadPanTilt (panCenter - 0 * panDegAvg, tiltAngle, speed);
    }
  }

}
