option(TurnOneRound, (bool)(false) isMirror, (Pose2f)(Pose2f(0.7f, 0.f, 0.f)) speed)
{
//    Pose2f speed(.7f, 0.f, 0.f);
#ifdef TARGET_ROBOT
    float angleCorrected = 20_deg;
#else
    float angleCorrected = 40_deg;
#endif
    if(isMirror) {
        speed = Pose2f(-0.7f, 0.f, 0.f);
    }

 Vector2f refPoint = Vector2f(4500.f,0.f);
 Vector2f refPointRelative;
   refPointRelative = theRobotPose.inverse() * refPoint;
 Angle refAngle;
     refAngle = refPointRelative.angle(); //机器人坐标系 球门的角度

  initial_state(first)
  {
      transition
      {
        if(state_time > 1000 && theWalkingEngineOutput.actionFlag == 0)
            goto second;
      }
      action
      {
          WalkToTarget(speed,Pose2f(90_deg + angleCorrected,0.f,0.f));  //@YY
//    	    theHeadControlMode = state_time % 6000 < 3000? HeadControl::lookLeftAndRightDown : HeadControl::lookLeftAndRightFast;
//    	    WalkToTarget(speed,Pose2f(refAngle,0.f,0.f));
      }
  }

  state(second)
  {
      transition
      {
        if(state_time > 1000 && theWalkingEngineOutput.actionFlag == 0)
            goto third;
      }
      action
      {
          WalkToTarget(speed,Pose2f(91_deg + angleCorrected,0.f,0.f));
      }
  }

  state(third)
  {
      transition
      {
        if(state_time > 1000 && theWalkingEngineOutput.actionFlag == 0)
            goto fourth;
      }
      action
      {
          WalkToTarget(speed,Pose2f(92_deg + angleCorrected,0.f,0.f));
      }
  }

  state(fourth)
  {
      transition
      {
        if(state_time > 1000 && theWalkingEngineOutput.actionFlag == 0)
            goto finish;
      }
      action
      {
          WalkToTarget(speed,Pose2f(93_deg + angleCorrected,0.f,0.f));
      }
  }

  target_state(finish)
  {
      action
      {
          WalkToTarget(speed,Pose2f(0.f,0.f,0.f));
      }
  }
}
