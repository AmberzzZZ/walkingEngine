option(KeeperOld)
{

	Pose2f target;
	bool kick;
	bool dangerousBall;
	bool keeperDive;
	common_transition
	{
		DECLARE_DEBUG_DRAWING("module:Behavior:Field", "drawingOnField");
		dangerousBall = keeper.ballDangerous();
		keeperDive = keeper.ifDiveNeeded();
		target = keeper.keeperDefencePosition();
		kick = keeper.keeperKickBallFarAway();
		if(ball.notSeenTime()>thePenaltyShootParameter.ballNotSeen)
		kick = false;
//		COMPLEX_DRAWING("module:Behavior:Field",
//		  {
//				DOT("module:Behavior:Field", target.translation.x(), target.translation.y(), ColorRGBA(0xaa, 0xff, 0xaa), ColorRGBA(0xaa, 0xff, 0xaa));
//		  });
		//std::cout<<"validity: "<<theRobotPose.validity<<"  robotx: "<<robot.x<<" roboty: "<<robot.y<<" robotr: "<<robot.rotation<<std::endl;
		if(keeper.turnAround())
		{
			goto location;
		}
		if(theFallDownState.state != FallDownState::upright && theFallDownState.state != FallDownState::undefined)
				goto lying;
	}
	initial_state(set)
	{
		transition
		{
			if(dangerousBall && ball.wasSeen())
			{
//				if(keeperDive)
//				{
//					if(ball.yPosWhenBallReachesOwnYAxis()>0)
//						goto diveLeft;
//					else
//						goto diveRight;
//				}
//				else
//				{
					if(ball.yPosWhenBallReachesOwnYAxis()>0)
						goto spreadLegLeft;
					else
						goto spreadLegRight;
//				}
			}
			else
			{
				if(kick)
					goto walkToBall;
				else if((target.translation.y()<30&&target.translation.y()>-30)&&(target.translation.x()<30&&target.translation.x()>-30))
					goto done;
			}
		}
		action
		{
//			TEAM_OUTPUT(idKeeperKick, bin, false);
      behavior.keeperOutput.isControlBall = false;
			if(!ball.wasSeen()&&ball.notSeenTime()>thePenaltyShootParameter.ballNotSeen)
				KeeperHeadControl(0, 3000, 3000, 0);
			else if(ball.x > 4000)
			{
				KeeperHeadControl(1500, 3000, 3000, 0);
			}
			else if(ball.x > 1500)
			{
				KeeperHeadControl(5000, 0, 3000, 0);
			}
			else
				KeeperHeadControl(5000, 0, 2000, 0);
			if(robot.y > 1200 || robot.y < -1200)
			{
				GoToReadyPose(robot.keeperReadyPose);
			}
			else
			{
				WalkToTarget(Pose2f(thePenaltyShootParameter.speedR, thePenaltyShootParameter.speedX, thePenaltyShootParameter.speedY), target);
			}
		}
	}
	state(done)
	{
		transition
		{
			if(target.translation.y()>50||target.translation.y()<-50||target.translation.x()>50||target.translation.x()<-50 || kick)
				goto set;
		}
		action
		{
//			TEAM_OUTPUT(idKeeperKick, bin, false);
      behavior.keeperOutput.isControlBall = false;
			if(!ball.wasSeen()&&ball.notSeenTime()>thePenaltyShootParameter.ballNotSeen)
				KeeperHeadControl(0, 3000, 3000, 0);
			else if(ball.x > 4000)
			{
				KeeperHeadControl(1500, 3000, 3000, 0);
			}
			else if(ball.x > 1500)
			{
				KeeperHeadControl(5000, 0, 3000, 0);
			}
			else
				KeeperHeadControl(5000, 0, 2000, 0);
			Stand();
		}
	}
	state(walkToBall)
	{
		transition
		{
			if((ball.global.x() - theFieldDimensions.xPosOwnGroundline) * (ball.global.x() - theFieldDimensions.xPosOwnGroundline) + ball.global.y() * ball.global.y() > theFieldDimensions.yPosLeftPenaltyArea * theFieldDimensions.yPosLeftPenaltyArea + 800 * 800)
				goto set;
			else if(ball.x < 170.f && ball.y < 120.f && ball.y > -120.f)
				goto alignToBall;
		}
		action
		{
//			TEAM_OUTPUT(idKeeperKick, bin, true);
      behavior.keeperOutput.isControlBall = true;
			if(!ball.wasSeen()&&ball.notSeenTime()>thePenaltyShootParameter.ballNotSeen)
				theHeadControlMode = HeadControl::lookLeftAndRight;
			else
				theHeadControlMode = HeadControl::lookAtBall;
			WalkToTarget(Pose2f(thePenaltyShootParameter.speedR, thePenaltyShootParameter.speedX, thePenaltyShootParameter.speedY), Pose2f(-robot.rotation, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));
		}
	}

		state(alignToBall)
	{
		transition
		{
			if((ball.global.x() - theFieldDimensions.xPosOwnGroundline) * (ball.global.x() - theFieldDimensions.xPosOwnGroundline) + ball.global.y() * ball.global.y() > theFieldDimensions.yPosLeftPenaltyArea * theFieldDimensions.yPosLeftPenaltyArea + 800 * 800)
				goto set;
			else if(ball.x < 155.f && ball.y < 100.f && ball.y > -100.f)
				goto kickBall;
				else if(state_time>3000)
					goto kickBall;
		}
		action
		{
//			TEAM_OUTPUT(idKeeperKick, bin, true);
      behavior.keeperOutput.isControlBall = true;
			if(!ball.wasSeen()&&ball.notSeenTime()>thePenaltyShootParameter.ballNotSeen)
				theHeadControlMode = HeadControl::lookLeftAndRight;
			else
				theHeadControlMode = HeadControl::lookAtBall;
			WalkToTarget(Pose2f(thePenaltyShootParameter.speedR, thePenaltyShootParameter.speedX, thePenaltyShootParameter.speedY), Pose2f(-robot.rotation, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));
		}
	}

	state(kickBall)
	{
		transition
		{
//			if((ball.global.x() - theFieldDimensions.xPosOwnGroundline) * (ball.global.x() - theFieldDimensions.xPosOwnGroundline) + ball.global.y() * ball.global.y() > theFieldDimensions.yPosLeftPenaltyArea * theFieldDimensions.yPosLeftPenaltyArea + 800 * 800)
//				goto set;
			if(action_done || state_time > 5000)
				goto set;
		}
		action
		{
//			TEAM_OUTPUT(idKeeperKick, bin, true);
      behavior.keeperOutput.isControlBall = true;
			if(!ball.wasSeen()&&ball.notSeenTime()>thePenaltyShootParameter.ballNotSeen)
				theHeadControlMode = HeadControl::lookLeftAndRight;
			else
				theHeadControlMode = HeadControl::lookAtBall;
			if(ball.y > 0)
			{
			  Kick(false);
			}
			else
			{
			  Kick(true);
			}
		}
	}
//	state(diveLeft)
//	{
//		transition
//		{
//			if(theFallDownState.state != FallDownState::upright && theFallDownState.state != FallDownState::undefined)
//				goto lying;
//		}
//		action
//		{
//			SpecialAction(SpecialActionRequest::goalkeeperDefend);
//		}
//	}
//	state(diveRight)
//	{
//		transition
//		{
//			if(theFallDownState.state != FallDownState::upright && theFallDownState.state != FallDownState::undefined)
//				goto lying;
//		}
//		action
//		{
//			SpecialAction(SpecialActionRequest::goalkeeperDefend,1);
//		}
//	}
	state(spreadLegLeft)
	{
		transition
		{
			if((ball.speedRelative.x() == 0||ball.speedRelative.x()>0)&&(ball.global.x()>robot.x))
				goto set;
			if(state_time > 4000)
				goto set;
		}
		action
		{
			SpecialAction(SpecialActionRequest::keeperBlockLeft);
		}
	}
	state(spreadLegRight)
	{
		transition
		{
			if((ball.speedRelative.x() == 0||ball.speedRelative.x()>0)&&(ball.global.x()>robot.x))
				goto set;
			if(state_time > 4000)
				goto set;
		}
		action
		{
			SpecialAction(SpecialActionRequest::keeperBlockLeft,1);
		}
	}
	state(lying)
	{
		transition
		{
			if(state_time > 2000)
				goto getUp;
		}
		action
		{
			SpecialAction(SpecialActionRequest::playDead);
		}
	}
	state(getUp)
	{
		transition
		{
			if(action_done)
				goto location;
		}
		action
		{
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
//			TEAM_OUTPUT(idKeeperKick, bin, false);
      behavior.keeperOutput.isControlBall = false;
			KeeperLocation();
		}
	}
}
