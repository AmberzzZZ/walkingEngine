/** Sets all members of the MotionRequest representation for executing a TargetMode-WalkRequest
 *  (i.e. Walk to a \c target at a \c speed)
 *  @param speed Walking speeds, in percentage.
 *  @param target Walking target, in mm and radians, relative to the robot.
 */ 
option(WalkToTarget, (const Pose2f&) speed, (const Pose2f&) target, (bool)(false) soft)
{ 

	//check obstacles between ball and robot itself
	bool obstacleNearBy = false;//(obstacle.numObstacleNearby>0);
	/** Set the motion request. */
	Pose2f target_cal;
	Pose2f target_err;

	common_transition
	{
		float kp = 1.6f;
		float kd = 0.3f;
		target_err = target - robot.target_last;
		target_cal = Pose2f(target.rotation, kp * target.translation.x() + kd * target_err.translation.x(), kp * target.translation.y() + kd * target_err.translation.y());
		robot.target_last = target;

	}

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
			if (obstacleNearBy)
			theMotionRequest.walkRequest.mode = WalkRequest::pathMode;
			else
			theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
			if(target.translation.norm() < 100.f)
			    theMotionRequest.walkRequest.target = target_cal;
			else
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
			if (obstacleNearBy)
			theMotionRequest.walkRequest.mode = WalkRequest::pathMode;
			else
			theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
			if(target.translation.norm() < 100.f)
				theMotionRequest.walkRequest.target = target_cal;
			else
				theMotionRequest.walkRequest.target = target;
			theMotionRequest.walkRequest.speed = speed;
			theMotionRequest.walkRequest.kickType = WalkRequest::none;
			theMotionRequest.walkRequest.soft = soft;
		}
	}
}
