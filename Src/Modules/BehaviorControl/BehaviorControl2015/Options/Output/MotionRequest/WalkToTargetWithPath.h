/** Sets all members of the MotionRequest representation for executing a TargetMode-WalkRequest
 *  (i.e. Walk to a \c target at a \c speed)
 *  @param speed Walking speeds, in percentage.
 *  @param target Walking target, in mm and radians, relative to the robot.
 */
option(WalkToTargetWithPath, (const Pose2f&) speed, (const Pose2f&) target, (bool)(false) soft)
{
  /** Set the motion request. */
//	std::cout << "theMotionInfo.motion = " << theMotionInfo.motion <<std::endl;
  initial_state(setRequest)
  {
    transition
    {
      if(theMotionInfo.motion == MotionRequest::walk)
        goto requestIsExecuted;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::pathMode;
      theMotionRequest.walkRequest.target = target;
      theMotionRequest.walkRequest.speed = speed;
      theMotionRequest.walkRequest.kickType = WalkRequest::none;
      theMotionRequest.walkRequest.soft = soft;
    }
  }

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted)
  {
    transition
    {
      if(theMotionInfo.motion != MotionRequest::walk)
        goto setRequest;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::pathMode;
      theMotionRequest.walkRequest.target = target;
      theMotionRequest.walkRequest.speed = speed;
      theMotionRequest.walkRequest.kickType = WalkRequest::none;
      theMotionRequest.walkRequest.soft = soft;
    }
  }
}
