/**
 * runswift's kick
 * @param kickType The WalkRequest::KickType to be executed
 * @param kickPose The Pose to move to
 */
option(FastKick, ((WalkRequest) KickType) kickType, (const Pose2f&) kickPose, (bool)(false) flash)
{
	Vector2f vector2Goal = theRobotPose.inverse() * Vector2f(4500.f, 0.f);
  /** Set the motion request / kickType. */
  initial_state(launch)
  {  
    transition
    {
      // printf("%s%d%s%d\n", "kickType----> ", kickType, "   ", theMotionInfo.walkRequest.kickType);//);
      // if(theMotionInfo.motion == MotionRequest::walk && theMotionInfo.walkRequest.kickType == kickType)
      //   goto execute;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::dribbleMode;
      theMotionRequest.walkRequest.target = kickPose;
      theMotionRequest.walkRequest.speed = Pose2f(1.f, 1.f, 1.f);
      theMotionRequest.walkRequest.kickType = kickType;
      theMotionRequest.walkRequest.flash = flash;
      theMotionRequest.walkRequest.step = Pose2f(vector2Goal.angle(), 0.f, 0.f);
    }
  }

  /** Executes the kick */
  state(execute)
  {
    transition
    {
      if(theMotionInfo.walkRequest.kickType == WalkRequest::none)
        goto finished;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::dribbleMode;
      theMotionRequest.walkRequest.target = kickPose;
      theMotionRequest.walkRequest.speed = Pose2f(1.f, 1.f, 1.f);
      theMotionRequest.walkRequest.kickType = WalkRequest::none;
      theMotionRequest.walkRequest.flash = flash;
      theMotionRequest.walkRequest.step = Pose2f(vector2Goal.angle(), 0.f, 0.f);
    }
  }

  /** The kick has been executed */
  target_state(finished)
  {
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::dribbleMode;
      theMotionRequest.walkRequest.target = kickPose;
      theMotionRequest.walkRequest.speed = Pose2f(1.f, 1.f, 1.f);
      theMotionRequest.walkRequest.kickType = WalkRequest::none;
      theMotionRequest.walkRequest.flash = flash;
    }
  }
}
