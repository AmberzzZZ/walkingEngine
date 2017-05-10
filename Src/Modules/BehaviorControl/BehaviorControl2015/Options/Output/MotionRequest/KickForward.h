/*
 * File:   KickForward.h
 * Author: wzf
 *
 * Created on 9/23/2015, 10:52
 */

//option(KickForward,(KickRequest::KickMotionID) kickMotionType, (bool)(false) mirror )
option (KickForward, ((KickRequest) KickMotionID) kickMotionType, (bool) (false) mirror, (float) (9000) kickDis)
{
  /** Set the motion request. */
  initial_state (setRequest)
  {
    transition
    {
      if (theMotionInfo.motion == MotionRequest::kick)
	goto requestIsExecuted;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::kick;
      theMotionRequest.kickRequest.kickMotionType = kickMotionType;
      theMotionRequest.kickRequest.kickDis = kickDis;      
      theMotionRequest.kickRequest.mirror = mirror;
    }
  }

  /** The motion process has started executing the request. */
  target_state (requestIsExecuted)
  {
    transition
    {
      if (theMotionInfo.motion != MotionRequest::kick)
	goto setRequest;
    }
    action
    {
//      PlaySound("zhaoyun.wav");
      theMotionRequest.motion = MotionRequest::kick;
      theMotionRequest.kickRequest.kickMotionType = kickMotionType;
      theMotionRequest.kickRequest.kickDis = kickDis;       
      theMotionRequest.kickRequest.mirror = mirror;
    }
  }
}
