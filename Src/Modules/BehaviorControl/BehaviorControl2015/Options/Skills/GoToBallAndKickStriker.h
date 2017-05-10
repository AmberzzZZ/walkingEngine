option (GoToBallAndKickStriker, (Vector2f)(Vector2f(4500.f, 0.f)) target)//采取先对准球門再对准球的策略
{   
	float kickX = 220.f;
	float kickY = 40.f;
	bool useRightFoot;
	Angle angleToTarget = (theRobotPose.inverse() * target).angle();
	common_transition
	{

	}
	initial_state(start)
	{
		transition
		{
			// std::cout<<"y---------"<<common.between(ball.positionRobot.y(), -kickY -20.f, -kickY +20.f)<<",,,,,,"<<common.between(ball.positionRobot.y(), kickY -20.f, kickY + 20.f)<<",,,,,,,,"<<alignFuzzy.useLeftFoot<<std::endl;
			// std::cout<<"x---------"<<common.between(ball.positionRobot.x(), kickX-20 , kickX+20)<<",,,,,,"<<common.between(ball.positionRobot.x(), kickX-20, kickX +20)<<std::endl;
			// std::cout<<"angle-----"<<common.between(angleToTarget, -5_deg, 5_deg)<<",,,,,,"<<common.between(angleToTarget, -5_deg, 5_deg)<<std::endl;

			if(!alignFuzzy.useLeftFoot && common.between(ball.positionRobot.y(), -kickY -20.f, -kickY +20.f)
	          && common.between(ball.positionRobot.x(), kickX-20 , kickX+20) && common.between(angleToTarget, -5_deg, 5_deg) )
				goto kick;
			if(alignFuzzy.useLeftFoot && common.between(ball.positionRobot.y(), kickY -20.f, kickY + 20.f)
	          && common.between(ball.positionRobot.x(), kickX-20, kickX +20) && common.between(angleToTarget, -5_deg, 5_deg) )
				goto kick;
		}
		action
		{
//			behavior.strikerOutput.isRequestNotAllowed = true;
//			behavior.strikerOutput.isControlBall = true;
			theHeadControlMode = HeadControl::lookAtBall;
			AlignMent(target);
		}
	}

	state(alignBehindBall)
	{
		transition
		{
			if(ball.positionRobot.norm() > 450 || fabs(ball.positionRobot.angle()) > 70_deg)
				goto start;
			else if(!useRightFoot && common.between(ball.positionRobot.y(), kickY -15.f, kickY + 15.f)
					&& common.between(ball.positionRobot.x(), kickX-15, kickX +15)
					&& common.between(angleToTarget, -3_deg, 3_deg))
			goto kick;//using left foot
			else if(useRightFoot && common.between(ball.positionRobot.y(), -kickY -15.f, -kickY +15.f)
					&& common.between(ball.positionRobot.x(), kickX-15 , kickX+15)
					&& common.between(angleToTarget, -3_deg, 3_deg))
			goto kick;//using right foot
		}
		action
		{
			behavior.strikerOutput.isRequestNotAllowed = true;
			behavior.strikerOutput.isControlBall = true;
			theHeadControlMode = HeadControl::lookAtBall;
			Pose2f speed = Pose2f(0.5f, 0.7f, 0.7f);
			Pose2f TargetToalign1 = Pose2f(angleToTarget, ball.positionRobot.x() - kickX, ball.positionRobot.y() - kickY);
			Pose2f TargetToalign2 = Pose2f(angleToTarget, ball.positionRobot.x() - kickX, ball.positionRobot.y() + kickY);
			if(!useRightFoot)
				WalkToTarget(speed,Pose2f(TargetToalign1.rotation, TargetToalign1.translation.x(), TargetToalign1.translation.y()));
			if(useRightFoot)
				WalkToTarget(speed,Pose2f(TargetToalign2.rotation, TargetToalign2.translation.x(), TargetToalign2.translation.y()));
		}
	}
	state(kick)
	{
		transition
		{
//			std::cout<<"action-done---->"<<action_done<<std::endl;
			if(action_done)
			{
				goto start;
			}
		}
		action
		{
			behavior.strikerOutput.isRequestNotAllowed = true;
			behavior.strikerOutput.isControlBall = true;
			theHeadControlMode = HeadControl::lookForward;
//			if(ball.positionRobot.y() < 0.f)
//				useRightFoot = true;
//			else
//				useRightFoot = false;
			Kick(!alignFuzzy.useLeftFoot);
			// Stand();
		}
	}
}
