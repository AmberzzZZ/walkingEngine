//@LiShu
option(Keeper)
{

	Pose2f target;
	bool kick;
	bool dangerousBall;
	bool keeperDive;
	float maxDistanceToFieldBorder;
	Vector2f ballSpeedOnField;

	Geometry::Line BallToRobot = Geometry::Line(keeper.ballPositionRobot,
			keeper.ballSpeedRobot);
	Vector2f intersection;
	bool isIntersect = Geometry::getIntersectionOfLines(
									BallToRobot, Geometry::Line(Vector2f(0, 0), Vector2f(0, 1)),
									intersection);

	common_transition
	{
		DECLARE_DEBUG_DRAWING("module:Behavior:Field", "drawingOnField");
		dangerousBall = keeper.ballDangerous2();
//		std::cout<<"dangerousBall--->"<<dangerousBall<<std::endl;
		keeperDive = keeper.ifDiveNeeded2();
		target = keeper.keeperDefencePosition2();
//		std::cout<<"target--->"<<target.translation.x()<<std::endl;
		kick = keeper.keeperKickBallFarAway();
//		ballSpeedOnField = theRobotPose * ball.speedRobot - theRobotPose;
		if(ball.notSeenTime()>2000.f && !ball.isTeamPositionValid)
			kick = false;
		COMPLEX_DRAWING("module:Behavior:Field");//@DXG
	}
	initial_state(set)
	{
		transition
		{
//    	if((stopBall && (ball.wasSeen()||supporter.useTeamBall) && (ball.endPosition.x() < -500.f && ball.speedRobot.x() < -1000.f)) ||
//    									(ball.positionField.norm() < 800.f && ball.endPosition.x() < 200.f && ball.speedRobot.x() < -300.f))
//			std::cout<<"ball.speed.x()--->"<<ball.speedRobot.x()<<std::endl;
//					std::cout<<"dangerousBall--->"<<dangerousBall<<std::endl;
			if(dangerousBall && (ball.notSeenTime()<2000.f||ball.isTeamPositionValid)
					&& keeper.ballEndPositionRobot.x()<100.f && ball.speedRobot.x() < -500.f)
			{
				if (!isIntersect)
				{
					goto spreadLittle;
				}
//				if(keeperDive)
//				{
//					if(intersection.y() > 0.f)
//						goto diveLeft;
//					else
//						goto diveRight;
//				}
//				else
				{
					if(intersection.y() > 300.f)
						goto diveLeft;
					else if (intersection.y() < -300.f)
						goto diveRight;
					else
						goto spreadLittle;
				}
			}
			else
			{
//				std::cout<<"do not need to save"<<std::endl;
				if(kick)
					goto walkToBall;
				else if((target.translation.y() < 100.f && target.translation.y() > -100.f)
						&& (target.translation.x() < 100.f && target.translation.x() > -100.f) && fabs(target.rotation) < 5_deg)
					goto done;
			}
		}
		action
		{
			behavior.keeperOutput.isControlBall = false;
			if(!(ball.wasSeen()|| ball.isTeamPositionValid)  && ball.notSeenTime() >2000.f)
				KeeperHeadControl(0, 3000, 3000, 0);
//				LookLeftAndRightFast();
			else
			{
				if(ball.positionRobot.norm() > 3000.f)
					KeeperHeadControl(0, 2000.f, 0.f, 2000.f, false);
				else
					LookAtBall();
			}
			if(robot.y > 1200 || robot.y < -1200)
			{
				GoToReadyPose(robot.keeperReadyPose);
			}
			else
			{
				WalkToTarget(Pose2f(0.5f, 0.7f, 0.3f), target); //行走速度怎么得到的，是否需要修改？
			}
		}
	}
	state(done)
	{
		transition
		{
			if(target.translation.y() > 150.f || target.translation.y() < -150.f
					|| target.translation.x() > 150.f || target.translation.x() < -150.f
					|| target.rotation > 0.175f || target.rotation < -0.175f
					|| kick)
				goto set;
			if(!(ball.wasSeen()|| ball.isTeamPositionValid) && ball.notSeenTime() > 2000.f && state_time > 6000.f)
			{
				goto turnLeftToSearchForBall;
			}
			if(dangerousBall && (ball.notSeenTime()<2000.f||ball.isTeamPositionValid)
					&& keeper.ballEndPositionRobot.x()<100.f && ball.speedRobot.x() < -500.f)
			{
				if (!isIntersect)
				{
					goto spreadLittle;
				}
				//				if(keeperDive)
				//				{
				//					if(intersection.y() > 0.f)
				//						goto diveLeft;
				//					else
				//						goto diveRight;
				//				}
				//				else
				{
					if(intersection.y() > 300.f)
					goto diveLeft;
					else if (intersection.y() < -300.f)
					goto diveRight;
					else
					goto spreadLittle;
				}
			}
		}
		action
		{
			behavior.keeperOutput.isControlBall = false;
			if(!(ball.wasSeen()|| ball.isTeamPositionValid) && ball.notSeenTime() >2000.f)
//				KeeperHeadControl(0, 3000, 3000, 0);
				LookLeftAndRightFast();
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
//			std::cout<<"kick--->"<<kick<<std::endl;
			if((keeper.ballPositionField.x() - theFieldDimensions.xPosOwnGroundline)
					* (keeper.ballPositionField.x() - theFieldDimensions.xPosOwnGroundline)
					+ keeper.ballPositionField.y() * keeper.ballPositionField.y()
					> theFieldDimensions.yPosLeftPenaltyArea
					* theFieldDimensions.yPosLeftPenaltyArea + 800 * 800 || !kick)
			goto set;
		}
		action
		{
			behavior.keeperOutput.isControlBall = true;
			if(!ball.wasSeen()&&ball.notSeenTime() > 2000.f)
				theHeadControlMode = HeadControl::lookLeftAndRight;
			else
				theHeadControlMode = HeadControl::lookAtBall;
			GoToBallAndKickKeeper();
		}
	}

	state(diveLeft)
	{
		transition
		{
			if(theFallDownState.state != FallDownState::upright
					&& theFallDownState.state != FallDownState::undefined)
			goto lying;
		}
		action
		{
//			SpecialAction(SpecialActionRequest::goalkeeperDefend);
			LookForward();
			SpecialAction(SpecialActionRequest::goalkeeperDefend,true);
		}
	}

	state(diveRight)
	{
		transition
		{
			if(theFallDownState.state != FallDownState::upright
					&& theFallDownState.state != FallDownState::undefined)
			goto lying;
		}
		action
		{
//			SpecialAction(SpecialActionRequest::goalkeeperDefend,1);
			LookForward();
			SpecialAction(SpecialActionRequest::goalkeeperDefend,false);
		}
	}

	state(spreadLegLeft)
	{
		transition
		{
			if((keeper.ballSpeedRobot.x() == 0 || keeper.ballSpeedRobot.x() > 0)
					&& (keeper.ballPositionField.x() > robot.x))
				goto set;
			if (keeper.ballEndPositionRobot.x()>350.f || intersection.y()>750.f)
				goto set;
			if(state_time > 4000)
				goto set;
		}
		action
		{
//			SpecialAction(SpecialActionRequest::goalkeeperDefend,true);
			LookForward();
			SpecialAction(SpecialActionRequest::sumoKeeper);
		}
	}

	state(spreadLegRight)
	{
		transition
		{
			if((keeper.ballSpeedRobot.x() == 0 || keeper.ballSpeedRobot.x() > 0)
					&& (keeper.ballPositionField.x()>robot.x))
				goto set;
			if (keeper.ballEndPositionRobot.x()>350.f || intersection.y()>750.f)
				goto set;
			if(state_time > 4000)
				goto set;
		}
		action
		{
//			SpecialAction(SpecialActionRequest::goalkeeperDefend,false);
			LookForward();
			SpecialAction(SpecialActionRequest::sumoKeeper);
		}
	}

	state(spreadLittle)
	{
		transition
		{
			if((keeper.ballSpeedRobot.x() == 0 || keeper.ballSpeedRobot.x() > 0)
					&& (keeper.ballPositionField.x() > robot.x))
				goto set;
			if (keeper.ballEndPositionRobot.x()>250.f || intersection.y()>750.f)
				goto set;
			if(state_time > 4000)
				goto set;
		}
		action
		{
			LookForward();
			SpecialAction(SpecialActionRequest::sumoKeeper);
//			SpecialAction(SpecialActionRequest::goalkeeperDefend,false);
		}
	}

	state(keeperStopBall)
	{
		transition
		{
			if((keeper.ballSpeedRobot.x() == 0 || keeper.ballSpeedRobot.x() > 0)
					&& (keeper.ballPositionField.x() > robot.x))
				goto set;
			if (keeper.ballEndPositionRobot.x()>250.f || intersection.y()>750.f)
				goto set;
			if(state_time > 4000)
				goto set;
		}
		action
		{
			SpecialAction(SpecialActionRequest::stopBall);
		}
	}

	state(lying)
	{
		transition
		{
			if(state_time > 2000)
				goto set;
		}
		action
		{
			LookForward();
			SpecialAction(SpecialActionRequest::playDead);
		}
	}

	state(getUp)
	{
		transition
		{
			if(action_done)
				goto set;
		}
		action
		{
			LookForward();
			GetUp();
		}
	}

	state(location)
	{
		transition
		{
			if(action_done)
			{
				goto set;
			}
		}
		action
		{
			behavior.keeperOutput.isControlBall = false;
			KeeperLocation();
		}
	}

	state(turnLeftToSearchForBall)
	{
		transition
		{
			if(ball.wasSeen() || ball.isTeamPositionValid)
			{
				goto set;
			}
			if(robot.rotation > 45_deg)
			{
				goto turnRightToSearchForBall;
			}
		}
		action
		{
			WalkAtSpeedPercentage(Pose2f(0.4f, 0.f, 0.f));
			SetHeadPanTilt (45_deg, 30_deg, 70_deg);
		}
	}
	state(turnRightToSearchForBall)
	{
		transition
		{
			if(ball.wasSeen() || ball.isTeamPositionValid)
			{
				goto set;
			}
			if(robot.rotation < -45_deg)
			{
				goto searchForBallEnd;
			}
		}
		action
		{
			WalkAtSpeedPercentage(Pose2f(-0.4f, 0.f, 0.f));
			SetHeadPanTilt (-45_deg, 30_deg, 70_deg);
		}
	}
	state(searchForBallEnd)
	{
		transition
		{
			if(ball.wasSeen() || ball.isTeamPositionValid)
			{
				goto set;
			}
			if(fabs(robot.rotation) < 5_deg)
			{
				goto set;
			}
		}
		action
		{
			WalkAtSpeedPercentage(Pose2f(0.4f, 0.f, 0.f));
			LookForward();
		}
	}
}
