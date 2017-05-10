option(Supporter)//supportor计算的target会在1500和-600之间震荡
{
	Pose2f target;
	Vector2f targetInWorld;
	bool stopBall;
	bool ifSpreadNeeded;
	Vector2f ballSuitablePositionRelative;
	float ballSuitableAngleRadRelativeToRobotAngle;
	bool robotAndBallCloseEnough;
	Pose2f globalKickBallPosition;
	const float kickX = 90.f;
	const float kickY = 40.f;
	static bool useRightFoot = false;

	Vector2f teammateBallError;
	Vector2f ballErrorWithTeammate;
	float ballToDefender;

	bool othersKick = false;
	Pose2f supporterPositionWhenKickAway;  //used when keeper or defender decide to kick the ball far away

	Vector2f intersectionOfBallToGoalAndRobot;

	float strikerBallTimeNotSeen = 0;
	for(const Teammate &teammate:theTeammateData.teammates)
	{
		if(teammate.role.role != Role::striker)
			continue;
		strikerBallTimeNotSeen = theFrameInfo.getTimeSince(teammate.ball.timeWhenLastSeen);
	}


	common_transition
	{
//		if((ball.positionRobot.norm() < teammate.striker.ballDistance - 500.f || strikerBallTimeNotSeen > 6000)&& ball.notSeenTime() < 1000)
//			behavior.supporterOutput.requestRoleType = Role::striker;
		if(!ball.isTeamPositionValid && ball.notSeenTime() > 2000.f)
		{
			PlaySound("lost.wav");
			goto searchForBall;
		}
		target = supporter.supporterPosition();
		targetInWorld = Geometry::relative2FieldCoord(theRobotPose, target.translation);
		stopBall = defender.toStopBall2()&& fabs(robot.rotation) < 110_deg;//当球往机器人滚动，并且最终位置在机器人的后方时需要停球
		ifSpreadNeeded = defender.ifSpreadNeeded2();

		robotAndBallCloseEnough = defender.robotAndBallCloseEnough();
		ballToDefender = teammate.ballRobotFromRobot(robot.numDefender).norm();

		globalKickBallPosition = Pose2f(0, ball.positionField.x() - 150.f, ball.positionField.y());

		if (behavior.defender.isControlBall == true || behavior.striker.isControlBall == true || behavior.keeper.isControlBall == true)//TODO:add the keeper.isControlBall
			othersKick = true;

		if(!useRightFoot && ball.positionRobot.y() < -50.f)
		{
			useRightFoot = true;
		}
		if(useRightFoot && ball.positionRobot.y() > 50.f)
		{
			useRightFoot = false;//这里也有use left of right foot
		}

		Vector2f lineDirection = Vector2f (ball.positionField.x () - 4500.f,
				ball.positionField.y () -0.f);
		Geometry::Line BallAndGoal = Geometry::Line (ball.positionField,
				lineDirection);
		Geometry::Line robotLine = Geometry::Line(Vector2f(robot.x, 0),
				Vector2f(0, 1));
		Geometry::getIntersectionOfLines (BallAndGoal, robotLine,intersectionOfBallToGoalAndRobot);
	}
	initial_state(set)
	{
		transition
		{
			if(stopBall && (ball.notSeenTime()<2000.f||ball.isTeamPositionValid)
					&& keeper.ballEndPositionRobot.x()<100.f && ball.speedRobot.x() < -500.f)
			{
				// TODO choose different actions depending on defender.yIfBallPassOwnAxis
				if(ifSpreadNeeded)
				{
					if (abs(defender.yIfBallPassOwnAxis) > 35.f
							&& abs(defender.yIfBallPassOwnAxis) <=200.f)
					goto spreadLittle;
					if(defender.yIfBallPassOwnAxis > 200.f)
					goto spreadLegLeft;
					else
					goto spreadLegRight;
				}
				else
				{
					if (abs(defender.yIfBallPassOwnAxis) > 35.f
							&& abs(defender.yIfBallPassOwnAxis) <=200.f)
					goto spreadLittle;
				}
			}
			if((othersKick && robot.x > defender.ballPositionField.x() && fabs(intersectionOfBallToGoalAndRobot.y() - robot.y) < 200.f))
			{
				goto walkAside;
			}
			else if(othersKick)
			{
				goto set;
			}

			if((ball.positionField.x() < -100.f ||
					((ball.positionRobot.norm() < 1200.f || (ball.positionField.x() - robot.x) < 1000.f)
					&&  ball.positionField.x() < 0.f))
					&&othersKick == false)
			{
				if (teammate.robotState(robot.numDefender)!=Teammate::INACTIVE && ball.positionField.x() > -2300.f)
					goto walkToBall;
				else if(teammate.robotState(robot.numDefender) == Teammate::INACTIVE)
					goto walkToBall;
			}
			if((target.translation.y() < 100 && target.translation.y() > -100)
						&& (target.translation.x() < 100 && target.translation.x() > -100)
						&& (target.rotation > -5_deg && target.rotation < 5_deg))
				goto done;
		}
		action
		{
			if(!(ball.wasSeen()|| ball.isTeamPositionValid) && ball.notSeenTime() > 2000.f)
				KeeperHeadControl(0, 3000, 3000, 0);
			else
			{
				if(ball.positionRobot.norm() > 3000.f)
					KeeperHeadControl(0, 2000.f, 0.f, 2000.f);
				else
					LookAtBall();
			}

			{
				WalkToTarget(Pose2f(0.4f, 0.7f, 0.5f), target);
//				WalkToPose_SiQiang(Pose2f(ball.angleRad, targetInWorld), Pose2f(5_deg, 100.f, 100.f));
			}
		}
	}
	state(done)
	{
		transition
		{
			if(ball.speedRobot.x() < -500.f)
				goto set;
			if((ball.positionField.x() < -100.f ||
					((ball.positionRobot.norm() < 1200.f || ball.endPositionRobot.x() < 800.f || (ball.positionField.x() - robot.x) < 1000.f)
					&&  ball.positionField.x() < 750.f))
					&&othersKick == false)
			{
				if (teammate.robotState(robot.numDefender)!=Teammate::INACTIVE
					|| ball.positionRobot.norm()<(ballToDefender+100))
				goto walkToBall;
			}
			if((target.translation.y() > 200 || target.translation.y() < -200
					  || target.translation.x() > 200 || target.translation.x() < -200
					  || target.rotation > 0.154f || target.rotation < -0.154f)  && !othersKick)
				goto set;
		}
		action
		{
			if(!(ball.wasSeen()|| ball.isTeamPositionValid) && ball.notSeenTime() > 2000.f)
				KeeperHeadControl(0, 3000, 3000, 0);
			else
			{
				if(ball.positionRobot.norm() > 3000.f)
					KeeperHeadControl(0, 2000.f, 0.f, 2000.f);
				else
					LookAtBall();
			}
			behavior.supporterOutput.isControlBall = false;
			Stand();
		}
	}

	state(walkToBall)
	{
		transition
		{
			if (othersKick)
				goto set;
			else if(ball.positionField.x()> 200.f)
				goto set;
			else if(ball.positionField.x() < -2500.f)
				goto set;
			else if(ball.speedRobot.x() < -500.f)
				goto set;
		}
		action
		{
			behavior.supporterOutput.isControlBall = false;
			GoToBallAndKickSupporter();
		}
	}

	state(standToDefence)
	{
		transition
		{
			if(behavior.keeper.isControlBall == false)
				goto set;
		}
		action
		{
			behavior.supporterOutput.isControlBall = false;
			if(!(ball.wasSeen()|| ball.isTeamPositionValid) && ball.notSeenTime() > 2000.f)
				KeeperHeadControl(0, 5000, 5000, 0);
			else
			{
				if(ball.positionRobot.norm() > 3000.f)
					KeeperHeadControl(0, 2000.f, 0.f, 2000.f);
				else
					LookAtBall();
			}
			Stand();
		}
	}

	state(spreadLegLeft)
	{
		transition
		{
			if(((ball.speedRobot.x() == 0 || ball.speedRobot.x() > 0)
							&& (ball.positionField.x() > robot.x)) || ball.positionField.x() <= robot.x)
				goto set;
			if(state_time > 2000)
				goto set;
		}
		action
		{
			behavior.supporterOutput.isControlBall = false;
			LookForward();
			SpecialAction(SpecialActionRequest::sumo,false);
		}
	}

	state(spreadLegRight)
	{
		transition
		{
			if(((ball.speedRobot.x() == 0 || ball.speedRobot.x() > 0)
				  && (ball.positionField.x() > robot.x)) || ball.positionField.x() <= robot.x)
				goto set;
			if(state_time > 2000)
				goto set;
		}
		action
		{
			behavior.supporterOutput.isControlBall = false;
			LookForward();
			SpecialAction(SpecialActionRequest::sumo, false);
		}
	}
	state(spreadLittle)
	{
		transition
		{
		  if(((ball.speedRobot.x() == 0 || ball.speedRobot.x() > 0)
				  && (ball.positionField.x() > robot.x)) || ball.positionField.x() <= robot.x)
			  goto set;
		  if(state_time > 2000)
			  goto set;
		}
		action
		{
			behavior.supporterOutput.isControlBall = false;
			LookForward();
			SpecialAction(SpecialActionRequest::sumo,false);
		}
	}
	state(searchForBall)
	{
		transition
		{
			if (ball.wasSeen() || ball.isTeamPositionValid)
				goto set;
		}
		action
		{
			SearchForBallFast();
		}
	}
	state(walkAside)
	{
		transition
		{
			if((!othersKick) &&( robot.x < ball.positionField.x() || fabs(intersectionOfBallToGoalAndRobot.y() - robot.y) > 300.f))
			{
				goto set;
			}
			else if(action_done)
				goto standAside;
		}
		action
		{
			behavior.supporterOutput.isControlBall = false;
			if(intersectionOfBallToGoalAndRobot.y() > robot.y)
				WalkToPose(Pose2f(0.f, robot.x, intersectionOfBallToGoalAndRobot.y() - 400.f), Pose2f(3_deg, 100.f, 100.f), true);
			else
				WalkToPose(Pose2f(0.f, robot.x, intersectionOfBallToGoalAndRobot.y() + 400.f), Pose2f(3_deg, 100.f, 100.f), true);
		}
	}
	state(standAside)
	{
		transition
		{
			if((!othersKick) &&( robot.x < ball.positionField.x() || fabs(intersectionOfBallToGoalAndRobot.y() - robot.y) > 300.f))
			{
				goto set;
			}
		}
		action
		{
			behavior.supporterOutput.isControlBall = false;
			if(!(ball.wasSeen()|| ball.isTeamPositionValid) && ball.notSeenTime() > 2000.f)
				KeeperHeadControl(0, 5000, 5000, 0);
			else
			{
				if(ball.positionRobot.norm() > 3000.f)
					KeeperHeadControl(0, 2000.f, 0.f, 2000.f);
				else
					LookAtBall();
			}
		}
	}
}
