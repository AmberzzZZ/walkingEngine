option(AvoidObstacleNearby)
{
	initial_state(readyForAvoid)
	{
		transition
		{
			if(obstacle.numObstacleNearby > 1)
				goto avoidByAllArms;
			else if(obstacle.numObstacleNearby > 0)
				if(obstacle.y < -100)
					goto avoidByRightArm;
				else
					goto avoidByLeftArm;
			else
				goto avoidSuccess;
		}
	}

	state(avoidByAllArms)
	{
		transition
		{
			if (state_time > 6000)
				goto avoidSuccess;
		}
		action
		{
			KeyFrameArms(ArmKeyFrameRequest::back);
		}

	}

	state(avoidByRightArm)
	{
		transition
		{
			if (state_time > 6000)
				goto avoidSuccess;
		}
		action
		{
			KeyFrameRightArm(ArmKeyFrameRequest::back);
		}

	}

	state(avoidByLeftArm)
	{
		transition
		{
			if (state_time > 6000)
				goto avoidSuccess;
		}
		action
		{
			KeyFrameLeftArm(ArmKeyFrameRequest::back);
		}

	}

	state(avoidSuccess)
	{
		transition
		{
				goto readyForAvoid;
		}

	}


}



