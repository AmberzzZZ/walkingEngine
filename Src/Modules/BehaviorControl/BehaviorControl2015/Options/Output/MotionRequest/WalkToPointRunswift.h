option(WalkToPointRunswift, (const Pose2f&) speed, (const Pose2f&) Pose, (bool)(false) soft)
{
	Vector2f vectorToTarget;
	Angle facingTurn;
	float distance;
	float forward, left;
	Angle heading;
	common_transition
	{
		vectorToTarget = theRobotPose.inverse() * Pose.translation;
		facingTurn = Pose.rotation - theRobotPose.rotation;
		facingTurn.normalize();
		distance = vectorToTarget.norm();
		heading = vectorToTarget.angle();
		if(distance < 400.f)
		{
			forward = vectorToTarget.x() * cos(-facingTurn) - vectorToTarget.y() * sin(-facingTurn);
			left = vectorToTarget.x() * sin(-facingTurn) + vectorToTarget.y() * cos(-facingTurn);
		}
		else
		{
			forward = vectorToTarget.x();
			left = vectorToTarget.y();
		}
	}
	initial_state(start)
	{
		transition
		{
			goto farAwayAndFacingAway;
		}
	}
	state(farAway)
	{
		transition
		{
			if(distance < 400.f)
			{
				goto nearBy;
			}
			else if(vectorToTarget.angle() > 90_deg)
			{
				goto farAwayAndFacingAway;
			}
		}
		action
		{
			common.stepClipped(forward, left, heading);
			theMotionRequest.motion = MotionRequest::walk;
			theMotionRequest.walkRequest.mode = WalkRequest::pointMode;
			theMotionRequest.walkRequest.speed = speed;
			theMotionRequest.walkRequest.kickType = WalkRequest::none;
			theMotionRequest.walkRequest.soft = soft;
			theMotionRequest.walkRequest.step = Pose2f(heading, forward, left);
		}
	}
	state(nearBy)
	{
		transition
		{
			if(distance > 600.f)
			{
				goto farAway;
			}
		}
		action
		{
			common.stepClipped(forward, left, facingTurn);
			theMotionRequest.motion = MotionRequest::walk;
			theMotionRequest.walkRequest.mode = WalkRequest::pointMode;
			theMotionRequest.walkRequest.speed = speed;
			theMotionRequest.walkRequest.kickType = WalkRequest::none;
			theMotionRequest.walkRequest.soft = soft;
			theMotionRequest.walkRequest.step = Pose2f(facingTurn, forward, left);
		}
	}
	state(farAwayAndFacingAway)
	{
		transition
		{
			if(vectorToTarget.angle() < 20_deg)
			{
				goto farAway;
			}
		}
		action
		{
			common.stepClipped(forward, left, heading);
			theMotionRequest.motion = MotionRequest::walk;
			theMotionRequest.walkRequest.mode = WalkRequest::pointMode;
			theMotionRequest.walkRequest.speed = speed;
			theMotionRequest.walkRequest.kickType = WalkRequest::none;
			theMotionRequest.walkRequest.soft = soft;
			theMotionRequest.walkRequest.step = Pose2f(heading, 0.f, 0.f);
		}
	}
}
