//@DXG
option(KeeperHeadControl, (int)(-1)lookScanTime, (int)(-1)lookLeftAndRightTime,
       (int)(-1)lookDownLeftAndRightTime, (int)(-1)lookAtBallTime, (bool)(false) isSearchForBall)
{
  initial_state(look_scan)
  {
    transition
    {
//      if(state_time > lookScanTime)
//        goto look_leftAndRight;
      if(lookScanTime == -1)
      {
        if(common.between(theJointAngles.angles[Joints::headYaw],-101_deg,-97_deg) && state_time > 2000) /** Do not change parameters arbitrarily @wzf*/
          goto look_leftAndRight;
      }
      else if(state_time > lookScanTime)
      {
        goto look_leftAndRight;
      }
    }
    action
    {
      theHeadControlMode = HeadControl::lookScan;
    }
  }
  state(look_leftAndRight)
  {
    transition
    {
      if(lookLeftAndRightTime == -1)
      {
        if(common.between(theJointAngles.angles[Joints::headYaw],-36_deg, -32_deg) && state_time > 2000) /** Do not change parameters arbitrarily @wzf*/
          goto look_downLeftAndRight;
      }
      else if(state_time > lookLeftAndRightTime)
      {
        goto look_downLeftAndRight;
      }
    }
    action
    {
//    	if(!isSearchForBall)
		  theHeadControlMode = HeadControl::lookLeftAndRight;
//    	else
//			LookLeftAndRight(60_deg, 70_deg);
    }
  }
  state(look_downLeftAndRight)
  {
    transition
    {
      if(lookDownLeftAndRightTime == -1)
      {
        if(common.between(theJointAngles.angles[Joints::headYaw],-24_deg,-20_deg) && state_time > 500) /** Do not change parameters arbitrarily @wzf*/
          goto look_atBall;
      }
      else if(state_time > lookDownLeftAndRightTime)
      {
        goto look_atBall;
      }
    }
    action
    {
//    	if(!isSearchForBall)
		  theHeadControlMode = HeadControl::lookLeftAndRightDown;//@DXG
//    	else
//    	LookLeftAndRightDown(60_deg, 70_deg);
    }
  }
  state(look_atBall)
  {
    transition
    {
      if(lookAtBallTime == -1)
      {
        if(state_time > 3000)
          goto look_scan;
      }
      else if(state_time > lookAtBallTime)
      {
        goto look_scan;
      }
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
    }
  }
}
