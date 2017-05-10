option (GoToBallAndKickWithPath, (Vector2f)(Vector2f(4500.f, 0.f)) target) //采取先对准球門再对准球的策略
{
	bool useRightFoot;
	bool pass;
//#ifdef TARGET_ROBOT
//  const float kickX = 240.f;
//  const float kickY = 40.f;
//#else
//  const float kickX = 170.f;
//  const float kickY = 40.f;
//#endif
//  const float kickX = 230.f;
//    const float kickY = 45.f;
	float kickX = 230.f;
	float kickY = 45.f;
//check obstacles between ball and robot itself
	bool obstacleNearBy = (obstacle.numObstacleNearby>0);

//  static float y_error_sum = 0;
	if(ball.positionRobot.y() < 0.f)
	useRightFoot = true;
	else
	useRightFoot = false;

	Angle angleToTarget = (theRobotPose.inverse() * target).angle();
	float minAngle = 10.f;
	float disToBall = ball.positionRobot.norm();

	common_transition
	{
		if (thePlayersPercept.isRobotNear && disToBall<500.f && ball.notSeenTime()<4000.f)
		{
			for (const PlayersPercept::Player& p : thePlayersPercept.players)
			{
				if (p.centerOnField.norm()<1000.f)
				{
					//					std::cout<<"nearRobot"<<std::endl;
					float angle=p.centerOnField.angle();
					DRAWTEXT("module:BEHAVIOR:nearRobot", p.x1FeetOnly,
							p.y2-20, 20, ColorRGBA(0, 0, 255, 255),
							"angle="<<angle
					);
					DRAWTEXT("module:BEHAVIOR:nearRobot", p.x1FeetOnly,
							p.y2, 20, ColorRGBA(255, 0, 0, 255),
							"dis="<<p.centerOnField.norm();
					);
					if (fabs(angle)<minAngle)
					{
						minAngle = angle;
					}
				}
			}
			//			goto test;
		}

		if (minAngle<1.7 && theRobotPose.translation.x() < 3200)
			pass = true;
		else
			pass = false;
	}


	initial_state(start)
	{
		transition
		{
//      if(action_done) //|| (ball.positionRobot.norm() < 300 && fabs(goals.angleToGoal - ball.positionRobot.angle()) < 60_deg))
//      {
//        if((target - ball.position).norm() < 500)
//            goto dribble;
//        else
//            goto alignToGoal;
//      }
				goto goToShootPoint;
		}
//    action
//    {
//      GoToShootPoint(target);
//    }
	}



	state(kick)
	{
		transition
		{
//    	if (pass)
//    	    	    		goto turnPass;
			if(action_done || state_time > 3000)
			goto start;
		}
		action
		{
			behavior.strikerOutput.isRequestNotAllowed = true;
			behavior.strikerOutput.isControlBall = true;
			theHeadControlMode = HeadControl::lookForward;
			if(robot.pose.translation.x() < -1500) //当机器人在比较后方的时候，就使用小一点的力气将球踢到前场。
			{
				Kick(useRightFoot);
			}
			else
			{
				Kick(useRightFoot);
			}
		}
	}

	state(dribble)
	{
		transition
		{
			if(action_done || ball.positionRobot.norm() > 600 || fabs(ball.positionRobot.angle()) > 90_deg)
			goto start;
		}
		action
		{
			Dribble(target.x(), target.y());
		}
	}

	state(goToShootPoint)
	{
		transition
		{
//			if(!ball.isTeamPositionValid && ball.notSeenTime() > 4000)
//			{
//				goto start;
//			}
//			if(action_done) //|| (ball.positionRobot.norm() < 300 && fabs(goals.angleToGoal - ball.positionRobot.angle()) < 60_deg))
//			{
//				if((target - ball.positionRobot).norm() < 500)
//				goto dribble;
//			}
		}
		action
		{
			GoToShootPointWithPath(target);
		}
	}
}



//  state(avoidShootPath)
//  {
//    transition
//    {
//      if(!behavior.othersControlBall)
//        goto start;
//    }
//    action
//    {
//      AvoidShootPath();
//    }
//  }

//state(alignToGoal)
//{
//	Pose2f turnSpeed(0.4f, 0.f, 0.f);
//	transition
//	{
//		if(ball.notSeenTime() > 4000)
//		{
//			PlaySound("lost.wav");
//			goto searchForBall;
//		}
//		else if(ball.positionRobot.norm() > 600 || fabs(ball.positionRobot.angle()) > 70_deg) //先走到shootpoint，然后进行球门的对准
//		goto goToShootPoint;
////			else if(fabs(angleToTarget) < 5_deg)
//		goto alignBehindBall;
//	}
//	action //告诉队友自己已经控制球了，不允许其他人切换我的角色
//	{
//		behavior.strikerOutput.isRequestNotAllowed = behavior.supporterOutput.isRequestNotAllowed = behavior.defenderOutput.isRequestNotAllowed = behavior.stabberOutput.isRequestNotAllowed = true;
//		behavior.strikerOutput.isControlBall = behavior.supporterOutput.isControlBall = behavior.defenderOutput.isControlBall = behavior.stabberOutput.isControlBall = true;
//		theHeadControlMode = HeadControl::lookAtBall;
//		if(angleToTarget >= 0.f)
//		WalkAtSpeedPercentage(turnSpeed);
//		else
//		WalkAtSpeedPercentage(-turnSpeed);
//
//	}
//}
//
//state(alignBehindBall)
//{
//	transition
//	{
//		if(pass)
//		{
//			kickX = 180.f;
//		}
//		else
//		{
//			kickX = 230.f;
//		}
//		if(ball.notSeenTime() > 4000)
//		{
//			PlaySound("lost.wav");
//			goto searchForBall;
//		}
//		else if(ball.positionRobot.norm() > 600 || fabs(ball.positionRobot.angle()) > 70_deg)
//			goto goToShootPoint;
//		else if(fabs(angleToTarget) > 10_deg)
//			goto alignToGoal;
//		else if(!useRightFoot && common.between(ball.positionRobot.y(), kickY -15.f, kickY + 15.f)
//				&& common.between(ball.positionRobot.x(), kickX -15.f, kickX + 15.f) )
//			goto beforeKick; //using left foot
//		else if(useRightFoot && common.between(ball.positionRobot.y(), -kickY -15.f, -kickY +15.f)
//				&& common.between(ball.positionRobot.x(), kickX -15.f, kickX + 15.f) )
//			goto beforeKick;//using right foot
////    	else if(state_time > 3000)
////    		goto kick;
//	}
//	action
//	{
//		behavior.strikerOutput.isRequestNotAllowed = behavior.supporterOutput.isRequestNotAllowed = behavior.defenderOutput.isRequestNotAllowed = behavior.stabberOutput.isRequestNotAllowed = true;
//		behavior.strikerOutput.isControlBall = behavior.supporterOutput.isControlBall = behavior.defenderOutput.isControlBall = behavior.stabberOutput.isControlBall = true;
//		theHeadControlMode = HeadControl::lookAtBall;
//		if(ball.positionRobot.y() >= 0)
//		{
//		if (obstacleNearBy)
//		WalkToTargetWithPath(Pose2f(.1f, .5f, 0.5f),
//				Pose2f(0.f, ball.positionRobot.x() - kickX, ball.positionRobot.y() - kickY)); //考虑能否使用pid进行对球
//		else
//		WalkToTarget(Pose2f(.1f, .5f, 0.5f),
//				Pose2f(0.f, ball.positionRobot.x() - kickX, ball.positionRobot.y() - kickY));//考虑能否使用pid进行对球
//		}
//		if(ball.positionRobot.y()<0)
//		{
//		if (obstacleNearBy)
//		WalkToTargetWithPath(Pose2f(.1f, .5f, 0.5f),
//				Pose2f(0.f, ball.positionRobot.x() - kickX, ball.positionRobot.y() + kickY));
//		else
//		WalkToTarget(Pose2f(.1f, .5f, 0.5f),
//				Pose2f(0.f, ball.positionRobot.x() - kickX, ball.positionRobot.y() + kickY));
//		}
//	}
//}
//state(alignToBall)
//{
//	transition
//	{
//		if(!useRightFoot && common.between(ball.positionRobot.y(), kickY -10.f, kickY + 10.f)
//				&& common.between(ball.positionRobot.x(), kickX -10.f, kickX + 10.f) && state_time > 1500)
//		goto kick; //using left foot
//		else if(useRightFoot && common.between(ball.positionRobot.y(), -kickY -10.f, -kickY +10.f)
//				&& common.between(ball.positionRobot.x(), kickX -10.f, kickX + 10.f) && state_time > 1500)
//		goto beforeKick;//using right foot
//	}
//	action
//	{
//		if(ball.positionRobot.y() >= 0)
//		{
//		if (obstacleNearBy)
//		WalkToTargetWithPath(Pose2f(0.1f, .5f, 0.3f),
//				Pose2f(0.f, ball.positionRobot.x() - kickX, ball.positionRobot.y() - kickY)); //考虑能否使用pid进行对球
//		else
//		WalkToTarget(Pose2f(0.1f, .5f, 0.3f),
//				Pose2f(0.f, ball.positionRobot.x() - kickX, ball.positionRobot.y() - kickY));//考虑能否使用pid进行对球
//		}
//		if(ball.positionRobot.y()<0)
//		{
//		if (obstacleNearBy)
//		WalkToTargetWithPath(Pose2f(0.1f, .5f, 0.3f),
//				Pose2f(0.f, ball.positionRobot.x() - kickX, ball.positionRobot.y() + kickY));
//		else
//		WalkToTarget(Pose2f(0.1f, .5f, 0.3f),
//				Pose2f(0.f, ball.positionRobot.x() - kickX, ball.positionRobot.y() + kickY));
//		}
//	}
//}
//state(turnPass)
//{
//	Pose2f turnSpeed1(0.8f, 1.f, 0.f);
//	Pose2f turnSpeed2(-0.8f, 1.f, 0.f);
//	transition
//	{
//		if ((minAngle>1.7f&&state_time>1000.f) || state_time>10000.f)
//		goto start;
//
//	}
//	action
//	{
//		PlaySound("yah.wav");
//		if (ball.notSeenTime()<4000.f)
//		theHeadControlMode = HeadControl::lookAtBall;
//		else
//		theHeadControlMode = HeadControl::lookLeftAndRightDown;
//		if (fabs(theRobotPose.translation.y())>2000.f)
//		{
//			if (fabs(minAngle)<0.4f)
//			{
//				if (theRobotPose.translation.y()<0)
//				WalkAtSpeedPercentage(turnSpeed1);
//				else
//				WalkAtSpeedPercentage(turnSpeed2);
//			}
//			else
//			WalkAtSpeedPercentage(Pose2f(0.f,0.8f,0.8f));
//
//		}
//		else
//		{
//			if (fabs(minAngle)<0.3f)
//			{
//				if (theRobotPose.translation.y()<0)
//				WalkAtSpeedPercentage(turnSpeed1);
//				else
//				WalkAtSpeedPercentage(turnSpeed2);
//			}
////  	  				WalkAtSpeedPercentage(turnSpeed1);
//			else
//			{
//				if (minAngle<0)
//				WalkAtSpeedPercentage(turnSpeed1);
//				else
//				WalkAtSpeedPercentage(turnSpeed2);
//			}
//		}
////  	  				WalkAtSpeedPercentage(turnSpeed2);
//	}
//}
//
//state(beforeKick)
//{
//	transition
//	{
//		if (pass)
//		goto turnPass;
//		if (state_time>500 && !pass)
//		goto kick;
//		if(action_done || state_time > 3000)
//		goto start;
//	}
//	action
//	{
//		behavior.strikerOutput.isRequestNotAllowed = behavior.supporterOutput.isRequestNotAllowed = behavior.defenderOutput.isRequestNotAllowed = behavior.stabberOutput.isRequestNotAllowed = true;
//		behavior.strikerOutput.isControlBall = behavior.supporterOutput.isControlBall = behavior.defenderOutput.isControlBall = behavior.stabberOutput.isControlBall = true;
//		theHeadControlMode = HeadControl::lookAtBall;
//
//	}
//}
