/*
 * @File  LookLeftOrRightWhileDribble.h
 * @Author dmx xf
 *@Usage  you need to add the path of  LookLeftOrRightWhileDribble.h to Src/Modules/BehaviorControl/BehaviorControl2013/Options.h
 * and  add state LookLeftOrRightWhileDribble to /Src/Modules/BehaviorControl/BehaviorControl2013/Options/HeadControl/HeadControl.h
 * @Date March 24th.2014
 */

/** Pun intended. Looks straight ahead in a way that the Nao V4's cameras cover the area in front of its feet as well as everything else in front of the robot.*/
option(LookLeftOrRightWhileDribble)
{

  /** Simply sets the necessary angles */
  initial_state(lookLeft)
  {
  transition
  {
    if(action_done || (ball.y<0 && state_time>2200) || (ball.y>0 && state_time>2200))
      goto lookRight;
  }
    action
    {
        if(ball.y<0)
          SetHeadPanTilt(40_deg, 15_deg, 40_deg);
        else SetHeadPanTilt(40_deg, 15_deg, 40_deg);
    }
  }

  state(lookRight)
  {
    transition
    {
      if(action_done || state_time>2200)
        goto lookLeft;
    }
    action
    {
        if(ball.y>0)
          SetHeadPanTilt(-40_deg, 15_deg, 40_deg);
        else SetHeadPanTilt(-40_deg, 15_deg, 40_deg);
    }

  }

}
