option(Defender)
{
	Pose2f target;
	bool stopBall;
	bool ifSpreadNeeded;
	Vector2f ballSuitablePositionRelative;
	float ballSuitableAngleRadRelativeToRobotAngle;
	bool robotAndBallCloseEnough;
	Pose2f globalKickBallPosition;
	const float kickX = 190.f;
	const float kickY = 45.f;
	bool useRightFoot = false;
	Pose2f defencePositionInFrontOfGoal;
	float ballToSupporter;
	bool othersKick;
	Vector2f intersectionOfBallToGoalAndRobot;

	common_transition
	{
		target = defender.defenderDefencePosition2();
		stopBall = defender.toStopBall2() && fabs(robot.rotation) < 110_deg;
		ifSpreadNeeded = defender.ifSpreadNeeded2();
		robotAndBallCloseEnough = defender.robotAndBallCloseEnough2();
		ballToSupporter = teammate.ballRobotFromRobot(robot.numSupporter).norm();

		globalKickBallPosition = Pose2f(0, ball.suitable.x() - 175.f, ball.suitable.y());
		if(behavior.supporter.isControlBall)
			othersKick = true;
		else
			othersKick = false;

		/*
		 * position when kepper try kick ball away.
		 */
		if (defender.ballPositionField.y()>0 && robot.y>0)
		{
			defencePositionInFrontOfGoal = Pose2f(
					std::atan2(defender.ballPositionField.y() - robot.y, defender.ballPositionField.x() - robot.x),
//	  -4200.f, 1500.f);
					defender.ballPositionField.x()+200.f,defender.ballPositionField.y()+600.f);
		}
		if (defender.ballPositionField.y()>0 && robot.y<0)
		{
			defencePositionInFrontOfGoal = Pose2f(
					std::atan2(defender.ballPositionField.y() - robot.y, defender.ballPositionField.x() - robot.x),
//	  -4200.f, -375.f);
					defender.ballPositionField.x()+200.f,defender.ballPositionField.y()-600.f);
		}
		if(defender.ballPositionField.y()<0 && robot.y>0)
		{
			defencePositionInFrontOfGoal = Pose2f(
					std::atan2(defender.ballPositionField.y() - robot.y, defender.ballPositionField.x() - robot.x),
//	  -4200.f, 375.f);
					defender.ballPositionField.x()+200.f,defender.ballPositionField.y()+600.f);
		}
		else
		{
			defencePositionInFrontOfGoal = Pose2f(
					std::atan2(defender.ballPositionField.y() - robot.y, defender.ballPositionField.x() - robot.x),
//	  -4200.f, -1500.f);
					defender.ballPositionField.x()+200.f,defender.ballPositionField.y()-600.f);
		}

		/*
		 * whether using right foot to kick, should be determined be robot performance
		 */
		if(!useRightFoot && defender.ballPositionRobot.y() < -50.f)
		{
			useRightFoot = true;
		}
		if(useRightFoot && defender.ballPositionRobot.y() > 50.f)
		{
			useRightFoot = false;
		}

		Vector2f lineDirection = Vector2f (defender.ballPositionField.x () - 4500.f,
				defender.ballPositionField.y () -0.f);
		Geometry::Line BallAndGoal = Geometry::Line (defender.ballPositionField,
				lineDirection);
		Geometry::Line robotLine = Geometry::Line(Vector2f(robot.x, 0),
		                                               Vector2f(0, 1));
		Geometry::getIntersectionOfLines (BallAndGoal, robotLine,intersectionOfBallToGoalAndRobot);


		if (!ball.isTeamPositionValid && ball.notSeenTime()>2000.f)
		{
			PlaySound("lost.wav");
			goto searchBall;
		}
	}

	initial_state(set)
	{
		transition
		{
			/*
			 * used to determine whether a defender should intercept a ball
			 */
//			std::cout<<"ball speed---->"<<ball.speedRobot.x()<<std::endl;
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
//				else
//					goto spreadLittle;
			}

			if(behavior.keeper.isControlBall == true)
			{
				goto walkInFrontOfGoal;
			}
			if((othersKick && robot.x > defender.ballPositionField.x() && fabs(intersectionOfBallToGoalAndRobot.y() - robot.y) < 200.f))
			{
				goto walkAside;
			}
			else if(othersKick)
			{
				goto standAside;
			}
			/*
			 * determine when to steal a ball
			 */
			if (teammate.robotState(robot.numSupporter)==Teammate::INACTIVE && behavior.keeper.isControlBall == false)
			{
				if((defender.ballPositionField.x() < -2500.f || (defender.ballPositionRobot.norm()<1200.f && defender.ballPositionField.x() < -2200.f)) && !othersKick)
					goto walkToBall;
			}
			else
			{
				if ((defender.ballPositionField.x() < -2500.f || (defender.ballPositionRobot.norm()<1200.f && defender.ballPositionField.x() < -2200.f))
												&& defender.ballPositionRobot.norm()<(ballToSupporter-200)
												&& !othersKick && behavior.keeper.isControlBall == false)
					goto walkToBall;
			}
			if((target.translation.y() < 100 && target.translation.y() > -100)
					&& (target.translation.x() < 100 && target.translation.x() > -100)
					&& (target.rotation > -0.087f && target.rotation < 0.087f))
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

			behavior.defenderOutput.isControlBall = false;

			if (ball.notSeenTime() > 5000 && !defender.useTeamBall)
			{
				WalkToPose(Pose2f(0, -3200, -1100));
			}
			else
			{
				WalkToTarget(Pose2f(0.4f, 0.7f, 0.3f), target);
			}
		}
	}
	state(done)
	{
		transition
		{
			if((target.translation.y() > 200 || target.translation.y() < -200
					|| target.translation.x() > 200 || target.translation.x() < -200
					|| target.rotation > 10_deg || target.rotation < -10_deg
					|| defender.ballPositionField.x() < -2000) && !othersKick)
				goto set;
			if(ball.speedRobot.x() < -500.f)
				goto set;
		}
		action
		{
			behavior.defenderOutput.isControlBall = false;

			if(!(ball.wasSeen()|| ball.isTeamPositionValid) && ball.notSeenTime() > 2000.f)
				KeeperHeadControl(0, 3000, 3000, 0);
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

	state(walkToBall)
	{
		transition
		{
			if(behavior.keeper.isControlBall == true)
				goto walkInFrontOfGoal;

			if (othersKick)
				goto set;
			else if((defender.ballPositionField.x() > -2000))
				goto set;
			else if (!(ball.wasSeen() || defender.useTeamBall) && ball.notSeenTime()>2000.f)
				goto set;
		}
		action
		{
			behavior.defenderOutput.isControlBall = false;

			if(!ball.wasSeen()&&ball.notSeenTime() > 2000.f)
				theHeadControlMode = HeadControl::lookLeftAndRight;
			else
				theHeadControlMode = HeadControl::lookAtBall;

			GoToBallAndKickDefender();
		}
	}

	state(walkInFrontOfGoal)
	{
		transition
		{
			if(behavior.keeper.isControlBall == false)
				goto set;
			else if(action_done)
				goto standToDefence;
		}
		action
		{
			behavior.defenderOutput.isControlBall = false;
			WalkToPose(defencePositionInFrontOfGoal, Pose2f(3_deg, 100.f, 100.f), true);
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

			behavior.defenderOutput.isControlBall = false;
			if(!(ball.wasSeen()|| ball.isTeamPositionValid) && ball.notSeenTime() > 2000.f)
				KeeperHeadControl(0, 3000, 3000, 0);
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
			if(((defender.ballSpeedRobot.x() == 0 || defender.ballSpeedRobot.x() > 0)
							&& (defender.ballPositionField.x() > robot.x)) || defender.ballPositionField.x() <= robot.x)
			goto set;
			if (defender.ballEndPositionRobot.x()>250.f || defender.ballEndPositionRobot.y()>750.f)
			goto set;
			if(state_time > 2000)
			goto set;
		}
		action
		{
			behavior.defenderOutput.isControlBall = false;
			theHeadControlMode = HeadControl::lookForward;
			SpecialAction(SpecialActionRequest::sumo,false);
		}
	}

	state(spreadLegRight)
	{
		transition
		{
			if(((defender.ballSpeedRobot.x() == 0 || defender.ballSpeedRobot.x() > 0)
							&& (defender.ballPositionField.x() > robot.x)) || defender.ballPositionField.x() <= robot.x)
			goto set;
			if (defender.ballEndPositionRobot.x()>250.f || defender.ballEndPositionRobot.y()>750.f)
			goto set;
			if(state_time > 2000)
			goto set;
		}
		action
		{
			behavior.defenderOutput.isControlBall = false;
			theHeadControlMode = HeadControl::lookForward;
			SpecialAction(SpecialActionRequest::sumo, false);
		}
	}

	state(spreadLittle)
	{
		transition
		{
			if(((defender.ballSpeedRobot.x() == 0 || defender.ballSpeedRobot.x() > 0)
							&& (defender.ballPositionField.x() > robot.x)) || defender.ballPositionField.x() <= robot.x)
			goto set;
			if (defender.ballEndPositionRobot.x()>250.f || defender.ballEndPositionRobot.y()>750.f)
			goto set;
			if(state_time > 2000)
			goto set;
		}
		action
		{
			theHeadControlMode = HeadControl::lookForward;
			SpecialAction(SpecialActionRequest::sumo);
		}
	}

	state(searchBall)
	{
		transition
		{
			if (ball.wasSeen() || ball.isTeamPositionValid)
			goto set;
		}
		action
		{
			SearchForBall();
		}
	}
	state(walkAside)
	{
		transition
		{
			if((!othersKick) && (robot.x < defender.ballPositionField.x() || fabs(intersectionOfBallToGoalAndRobot.y() - robot.y) > 300.f))
			{
				goto set;
			}
			else if(action_done)
				goto standAside;
		}
		action
		{
			behavior.defenderOutput.isControlBall = false;
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
			if((!othersKick) && (robot.x < defender.ballPositionField.x() || fabs(intersectionOfBallToGoalAndRobot.y() - robot.y) > 300.f))
			{
				goto set;
			}
		}
		action
		{
			behavior.defenderOutput.isControlBall = false;
			if(!(ball.wasSeen()|| ball.isTeamPositionValid) && ball.notSeenTime() > 2000.f)
				KeeperHeadControl(0, 3000, 3000, 0);
			else
			{
				if(ball.positionRobot.norm() > 3000.f)
					KeeperHeadControl(0, 2000.f, 0.f, 2000.f);
				else
					theHeadControlMode = HeadControl::lookAtBall;
			}
			Stand();
		}
	}
}
