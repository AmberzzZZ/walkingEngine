option(SupporterOld)
{
	Pose2f target;
	bool stopBall;
	bool ifSpreadNeeded;
	Vector2f ballSuitablePositionRelative;
	float ballSuitableAngleRadRelativeToRobotAngle;
	bool robotAndBallCloseEnough;
	Pose2f globalKickBallPosition;
	const float kickX = 145.f;
	const float kickY = 40.f;
	bool useRightFoot;
	Pose2f defencePositionInFrontOfGoal;
	common_transition
	{
//		defender.yIfBallCanPassGroundLine();
//		defender.globalSpeed(ball.speedRelative);
//		defender.yIfBallCanPassGroundLine();
		ball.getSuitablePosition();
		target = defender.supporterDefencePosition(); //@XHan
		stopBall = defender.toStopBall();
		ifSpreadNeeded = defender.ifSpreadNeeded();
		robotAndBallCloseEnough = defender.robotAndBallCloseEnough();
		ballSuitablePositionRelative = Geometry::fieldCoord2Relative(Pose2f(robot.rotation, robot.x, robot.y), Vector2f(ball.suitable.x() - 150.f, ball.suitable.y()));
		globalKickBallPosition = Pose2f(0, ball.suitable.x() - 150.f, ball.suitable.y());
		ballSuitableAngleRadRelativeToRobotAngle = std::atan2(ball.suitable.y() - robot.y, ball.suitable.x() - robot.x) - robot.rotation;
		if(ball.suitable.y() > 0)
		{
			defencePositionInFrontOfGoal = Pose2f(std::atan2(ball.suitable.y() - robot.y, ball.suitable.x() - robot.x), -4200.f, -375.f);
		}
		else
		{
			defencePositionInFrontOfGoal = Pose2f(std::atan2(ball.suitable.y() - robot.y, ball.suitable.x() - robot.x), -4200.f, 375.f);
		}
		if(ball.y < 0.f || ball.y == 0.f)
		{
			useRightFoot = true;
		}
		else if(ball.y > 0.f)
		{
			useRightFoot = false;
		}
	}
	initial_state(set)
	{
		transition
		{
			if(stopBall && ball.wasSeen())
			{
				if(ifSpreadNeeded)
				{
					if(ball.yPosWhenBallReachesOwnYAxis() > 0)
						goto spreadLegLeft;
					else
						goto spreadLegRight;
				}
//				else
//					goto spreadLittle;
			}
			//@XHan
//			else if(theTeamMateData.keeperKick[1] == true)
//			{
//				goto walkInFrontOfGoal;
//			}
			else
			{
				if(ball.suitable.x() < -1500)
					goto walkToBall;
				else if((target.translation.y() < 30 && target.translation.y() > -30) && (target.translation.x() < 30 && target.translation.x() > -30) && (target.rotation > -0.087f && target.rotation < 0.087f))
					goto done;
			}
		}
		action
		{
//			if(!ball.wasSeen() && ball.notSeenTime() > thePenaltyShootParameter.ballNotSeen)
//				KeeperHeadControl(0, 3000, 3000, 0);
//			else if(ball.x > 4000)
//			{
//				KeeperHeadControl(1500, 3000, 3000, 0);
//			}
//			else if(ball.x > 1500)
//			{
//				KeeperHeadControl(5000, 0, 3000, 0);
//			}
//			else
//				KeeperHeadControl(5000, 0, 2000, 0);
			//TEAM_OUTPUT(idDefenderKick, bin, false);
			if(!ball.wasSeen() && ball.notSeenTime() > thePenaltyShootParameter.ballNotSeen)
				KeeperHeadControl(0, 3000, 3000, 0);
			else
				KeeperHeadControl(8000, 0, 3000, 0);
			if(robot.y > 1200 || robot.y < -1200)
			{
				GoToReadyPose(robot.defenderReadyPose+Pose2f(0.f,250.f,0.f));
			}
			else if(ball.notSeenTime() > 5000)
			{
				WalkToPose(Pose2f(0, -2500, 0));
			}
			else
			{
				WalkToTarget(Pose2f(0.4f,0.8f,0.8f), target);
			}
		}
	}
	state(done)
	{
		transition
		{
			if(target.translation.y() > 50 || target.translation.y() < -50 || target.translation.x() > 50 || target.translation.x() < -50 || target.rotation > 0.175f || target.rotation < -0.175f || ball.suitable.x() < -2000)
				goto set;
		}
		action
		{
//			if(!ball.wasSeen() && ball.notSeenTime() > thePenaltyShootParameter.ballNotSeen)
//				KeeperHeadControl(0, 3000, 3000, 0);
//			else if(ball.x > 4000)
//			{
//				KeeperHeadControl(1500, 3000, 3000, 0);
//			}
//			else if(ball.x > 1500)
//			{
//				KeeperHeadControl(5000, 0, 3000, 0);
//			}
//			else
//				KeeperHeadControl(5000, 0, 2000, 0);
                   // TEAM_OUTPUT(idDefenderKick, bin, false);
			if(!ball.wasSeen() && ball.notSeenTime() > thePenaltyShootParameter.ballNotSeen)
				KeeperHeadControl(0, 3000, 3000, 0);
			else
				KeeperHeadControl(8000, 0, 3000, 0);
			Stand();
		}
	}

	state(walkToBall)
	{
		transition
		{
//			if(theTeamMateData.keeperKick[1] == true)
//				goto walkInFrontOfGoal;
			if(ball.suitable.x() > -1200 || (stopBall && ball.wasSeen() && ifSpreadNeeded && fabs(robot.rotation) < 1.4f))
				goto set;
			else if(ball.distance < 650)
				goto goBehindBall;
		}
		action
		{
                   // TEAM_OUTPUT(idDefenderKick, bin, false);
			WalkToPose(globalKickBallPosition, Pose2f(3.f, 100.f, 100.f), true);
		}
	}

	state(goBehindBall)
	{
		transition
		{
			if(ball.distance > 650)
				goto walkToBall;
			else if(action_done)
				goto walkCloseToBall;
//			else if(theTeamMateData.keeperKick[1] == true)
//				goto walkInFrontOfGoal;
			else if(ball.suitable.x() > -1000)
				goto set;
		}
		action
		{
                    //TEAM_OUTPUT(idDefenderKick, bin, true);
                    GoBehindBall(4000, 0);
		}
	}
/*
	state(walkInFrontOfGoal)
	{
		transition
		{
			if(theTeamMateData.keeperKick[1] == false)
				goto set;
			else if(action_done)
				goto standToDefence;
		}
		action
		{
                    //TEAM_OUTPUT(idDefenderKick, bin, false);
                    WalkToPose(defencePositionInFrontOfGoal, 100.f, 100.f, 100.f, 100.f, 3.f, true);
		}
	}

	state(standToDefence)
	{
		transition
		{
			if(theTeamMateData.keeperKick[1] == false)
				goto set;
		}
		action
		{

                    //TEAM_OUTPUT(idDefenderKick, bin, false);
                    if(!ball.wasSeen() && ball.notSeenTime() > thePenaltyShootParameter.ballNotSeen)
				KeeperHeadControl(0, 3000, 3000, 0);
			else
				KeeperHeadControl(8000, 0, 3000, 0);
			Stand();
		}
	}*/

	state(walkCloseToBall)
	{
		transition
		{
			if(ball.x < 200)
				goto alignToBall;
			else if(ball.speedRelative.x() != 0 || state_time > 7000)
				goto set;
		}
		action
		{
		//TEAM_OUTPUT(idDefenderKick, bin, true);
                    theHeadControlMode = (state_time % 4000) < 2000 ? HeadControl::lookLeftAndRightDown : HeadControl::lookAtBall;
			WalkToPose(globalKickBallPosition);
		}
	}

	state(alignToBall)
	{
		transition
		{
			if(ball.distance > 300)
				goto set;
			else if((common.between(theBallModel.estimate.position.y(), kickY -10.f, kickY + 10.f) && common.between(theBallModel.estimate.position.x(), kickX -10.f, kickX +10.f)) || state_time > 3000)
				goto kickBall;
		}
		action
		{
                    //TEAM_OUTPUT(idDefenderKick, bin, true);
                    theHeadControlMode = (state_time % 4000) < 2000 ? HeadControl::lookLeftAndRightDown : HeadControl::lookAtBall;
			if(useRightFoot)
		    {
				WalkToTarget(Pose2f(0.60f, 0.60f, 0.60f), Pose2f(-robot.rotation, ball.x - kickX, ball.y + kickY));
		    }
		    else
		    {
				WalkToTarget(Pose2f(0.60f, 0.60f, 0.60f), Pose2f(-robot.rotation, ball.x - kickX, ball.y - kickY));
		    }
		}
	}

	state(kickBall)
	{
		transition
		{
			if(action_done || state_time > 5000)
				goto set;
		}
		action
		{
         //               TEAM_OUTPUT(idDefenderKick, bin, true);
                    if(!ball.wasSeen()&&ball.notSeenTime()>thePenaltyShootParameter.ballNotSeen)
				theHeadControlMode = HeadControl::lookLeftAndRight;
			else
				theHeadControlMode = HeadControl::lookAtBall;
    Kick(useRightFoot);
		}
	}

	state(spreadLegLeft)
	{
		transition
		{
			if(((ball.speedRelative.x() == 0 || ball.speedRelative.x() > 0) && (ball.global.x() > robot.x)) || ball.global.x() <= robot.x)
				goto set;
			if(state_time > 2000)
				goto set;
		}
		action
		{
		//TEAM_OUTPUT(idDefenderKick, bin, false);
                    SpecialAction(SpecialActionRequest::keeperBlockLeft);
		}
	}

	state(spreadLegRight)
	{
		transition
		{
			if(((ball.speedRelative.x() == 0 || ball.speedRelative.x() > 0) && (ball.global.x() > robot.x)) || ball.global.x() <= robot.x)
				goto set;
			if(state_time > 2000)
				goto set;
		}
		action
		{
                   // TEAM_OUTPUT(idDefenderKick, bin, false);
			SpecialAction(SpecialActionRequest::keeperBlockLeft, 1);
		}
	}

//	state(spreadLittle)
//	{
//		transition
//		{
//			if(((ball.speedRelative.x() == 0 || ball.speedRelative.x() > 0) && (ball.global.x() > robot.x)) || ball.global.x() <= robot.x)
//				goto set;
//			if(state_time > 2000)
//				goto set;
//		}
//		action
//		{
//			SpecialAction(SpecialActionRequest::stopBall);
//		}
//	}
}
