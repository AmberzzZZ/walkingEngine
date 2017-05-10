/**
 * @file AvoidObstacle.h
 */
option(AvoidObstacle,(Vector2f) pose)
{
	Vector2f rePose=Geometry::fieldCoord2Relative(theRobotPose,pose);
	float obsWidth = 250.f;//obsWidth is  obstacles' width @hjq 670
//	if((ball.distance < 1000) || ((sqr(robot.x - pose.x()) + sqr(robot.y - pose.y())) < sqr(1000)))
//		obsWidth = 500.f;
//	if((fabs(ball.x - obstacle.x) < 400) || (fabs(Geometry::fieldCoord2Relative(theRobotPose,Vector2f(pose.x(),pose.y())).x()-obstacle.x)<400))
//		obsWidth = 300;//@hjq original 300
//	#ifdef TARGET_SIM
//		obsWidth = 330.f;
//		if((ball.distance<1000)||((sqr(robot.x-pose.x())+sqr(robot.y-pose.y()))<sqr(1000)))//1200 for debug
//		obsWidth = 250.f;
//		if((fabs(ball.x-obstacle.x)<400)||(fabs(Geometry::fieldCoord2Relative(theRobotPose,Vector2f(pose.x(),pose.y())).x()-obstacle.x)<400))
//		obsWidth = 150;
//	#endif
	initial_state(initial)
	{
		transition
		{
//			PlaySound("doh.wav");
			//if(walk.toAvoidNear)
			if(obstacle.x<500 && fabs(obstacle.y)<obsWidth && ((ball.positionRobot.x())>100 || (rePose.x())>100))
			{
				//if(walk.avoidToLeft)
				if(obstacle.y<-100)
				{
					goto avoid_left;
				}
				else
				{
					goto avoid_right;
				}
			}
			else if(state_time>500)
			{
				goto avoid_success;
			}
		}
	}
	state(avoid_left)
	{
		transition
		{
			//if(!walk.toAvoidNear)
			if(fabs(obstacle.y)>obsWidth && state_time>500)
				goto avoid_success;
			else if(state_time>4000)//@yongqi original5000@hjq5000
			{
				obstacle.x=0;
				obstacle.y=0;
				goto avoid_success;
			}
		}
		action
		{
//			KeyFrameRightArm(ArmKeyFrameRequest::back);
			if(obstacle.x<200) //@hjq original 200
//				WalkAtSpeed(Pose2f(0,0,35));
				WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.5f));
			else
//				WalkAtSpeed(Pose2f(0,5,35));//@original 0,5,35
//				WalkAtSpeedPercentage(Pose2f(0.f, 0.5f, 0.5f));
				WalkAtSpeedPercentage(Pose2f(0.f,0.3f,0.5f));
		}

	}
	state(avoid_right)
	{
		transition
		{
			//if(!walk.toAvoidNear)
			if(fabs(obstacle.y)>obsWidth && state_time>500)
				goto avoid_success;
			else if(state_time>4000)//@yongqi original5000@hjq6000
			{
				obstacle.x=0;
				obstacle.y=0;
				goto avoid_success;
			}
		}
		action
		{
//			KeyFrameLeftArm(ArmKeyFrameRequest::back);
			if(obstacle.x<200)//@hjq original 200
//				WalkAtSpeed(Pose2f(0,0,-35));
//				WalkAtSpeedPercentage(Pose2f(0.f, 0.f, -0.5f));
				WalkAtSpeedPercentage(Pose2f(0.f, 0.f, -0.5f));
			else
//				WalkAtSpeed(Pose2f(0,5,-35));//@original 0,5,-35
//				WalkAtSpeedPercentage(Pose2f(0.f, 0.5f, -0.5f));
				WalkAtSpeedPercentage(Pose2f(0.f,0.3f,-0.5f));
		}

	}


	target_state(avoid_success)
	{
		transition
		{
			goto initial;
		}
	}

}
