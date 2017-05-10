option(DribbleRunswift, (const bool) useRightFoot, (const Angle) turn)
{
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
			theMotionRequest.walkRequest.mode = WalkRequest::dribbleMode;
			theMotionRequest.walkRequest.kickType = WalkRequest::none;
			theMotionRequest.walkRequest.step = Pose2f(turn, 0.f, 0.f);
			theMotionRequest.DribbleFoot = useRightFoot;
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
			theMotionRequest.walkRequest.mode = WalkRequest::dribbleMode;
			theMotionRequest.walkRequest.kickType = WalkRequest::none;
			theMotionRequest.walkRequest.step = Pose2f(turn, 0.f, 0.f);
			theMotionRequest.DribbleFoot = useRightFoot;
		}
	}
}
