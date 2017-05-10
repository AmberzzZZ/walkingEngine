option(StabberOld)
{
	common_transition
  {
    if(theTeamBallModel.isValid == false)
			goto searchForBallTogether;
  }

	initial_state(start)
	{
		transition
		{
			if(ball.wasSeen()||ball.isTeamPositionValid)
				goto walkToStabber;
		}

		action
		{
			theHeadControlMode = HeadControl::lookLeftAndRight;
			Stand();
		}
	}

	state(walkToStabber)
	{
		transition
		{
			Pose2f target=stabber.StabberPosition();
			if(target.translation.x()<100&&target.translation.x()>-100&&target.translation.y()<100&&target.translation.y()>-100&&target.rotation<5*3.14f/180&&target.rotation>-5*3.14f/180)
				goto wait;
			if(ball.global.x()>3200)
				goto walkBack;
		}

		action
		{
			theHeadControlMode = HeadControl::lookLeftAndRight;
			Pose2f targetAbs;
			Pose2f target=stabber.StabberPosition();
			if(target.translation.norm()<1000)
			{
				WalkToTarget(Pose2f(0.4f,0.8f,0.8f),target);
			}
			else
			{
				targetAbs.translation=Geometry::relative2FieldCoord(Pose2f(robot.rotation, robot.x, robot.y),target.translation);
				targetAbs.rotation=target.rotation+robot.rotation;
				WalkToPose(targetAbs);
			}

		}

	}
	state(walkBack)
	{
		transition
		{
			Pose2f target=stabber.StabberPositionWhenBallNearOppoGoal();
			if(target.translation.x()<100&&target.translation.x()>-100&&target.translation.y()<100&&target.translation.y()>-100&&target.rotation<5*3.14f/180&&target.rotation>-5*3.14f/180)
				goto wait;
			if(ball.global.x()<2800)
				goto walkToStabber;
		}

		action
		{
			theHeadControlMode = HeadControl::lookLeftAndRight;
			Pose2f targetAbs;
			Pose2f target=stabber.StabberPositionWhenBallNearOppoGoal();
			if(target.translation.norm()<1000)
			{
				WalkToTarget(Pose2f(0.4f,0.8f,0.8f),target);
			}
			else
			{
				targetAbs.translation=Geometry::relative2FieldCoord(Pose2f(robot.rotation, robot.x, robot.y),target.translation);
				targetAbs.rotation=target.rotation+robot.rotation;
				WalkToPose(targetAbs);
			}

		}
	}
	state(wait)
	{
		transition
		{
			Pose2f target=stabber.StabberPosition();
			Pose2f targetBack=stabber.StabberPositionWhenBallNearOppoGoal();
			if(ball.global.x()<2800)
			{
				if(target.translation.x()>200.0||target.translation.x()<-200||target.translation.y()>200||target.translation.y()<-200)
				{
						goto walkToStabber;
				}
			}
			else if(ball.global.x()>3200)
			{
				if(targetBack.translation.x()>200||targetBack.translation.x()<-200||targetBack.translation.y()>200||targetBack.translation.y()<-200)
				{
						goto walkBack;
				}
			}


		}

		action
		{
			theHeadControlMode = HeadControl::lookLeftAndRight;
			Stand();
		}
	}

	state(searchForBallTogether)
  {
	  transition
	  {
		  if(ball.isTeamPositionValid)
			  goto start;

	  }
	  action
	  {
		  SearchForBallTogether();
	  }
  }

}
