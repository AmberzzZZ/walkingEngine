option (GoToBallAndKickSupporter)//采取先对准球門再对准球的策略
{
  bool useRightFoot;
//#ifdef TARGET_ROBOT
//  const float kickX = 240.f;
//  const float kickY = 40.f;
//#else
//  const float kickX = 170.f;
//  const float kickY = 40.f;
//#endif
  const float kickX = 220.f;
    const float kickY = 45.f;
//  static float y_error_sum = 0;
  if(ball.positionRobot.y() < 0.f)
    useRightFoot = true;
  else
    useRightFoot = false;
  Vector2f target = Vector2f(4500.f, 0.f);
  Angle angleToTarget = (theRobotPose.inverse() * target).angle();

  common_transition
  {

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

  state(alignToGoal)
  {
    Pose2f turnSpeed(0.4f, 0.f, 0.f);
    transition
    {
      if(ball.positionRobot.norm() > 600 || fabs(ball.positionRobot.angle()) > 70_deg)//先走到shootpoint，然后进行球门的对准
        goto goToShootPoint;
      else if(fabs(angleToTarget) < 5_deg)
        goto alignBehindBall;
    }
    action//告诉队友自己已经控制球了，不允许其他人切换我的角色
    {
        behavior.supporterOutput.isRequestNotAllowed = true;
        behavior.supporterOutput.isControlBall = true;
      theHeadControlMode = HeadControl::lookAtBall;
      if(angleToTarget >= 0.f)
        WalkAtSpeedPercentage(turnSpeed);
      else
        WalkAtSpeedPercentage(-turnSpeed);
    }
  }

  state(alignBehindBall)
  {
    transition
    {
    	if(ball.positionRobot.norm() > 600 || fabs(ball.positionRobot.angle()) > 70_deg)
    		goto goToShootPoint;
    	else if(fabs(angleToTarget) > 10_deg)
    		goto alignToGoal;
    	else if(!useRightFoot && common.between(ball.positionRobot.y(), kickY -15.f, kickY + 15.f)
    			&& common.between(ball.positionRobot.x(), kickX -15.f, kickX + 15.f) )
    		goto kick; //using left foot
    	else if(useRightFoot && common.between(ball.positionRobot.y(), -kickY -15.f, -kickY +15.f)
    			&& common.between(ball.positionRobot.x(), kickX -15.f, kickX + 15.f) )
    		goto kick; //using right foot
//    	else if(state_time > 3000)
//    		goto kick;
    }
    action
    {
        behavior.supporterOutput.isRequestNotAllowed = true;
        behavior.supporterOutput.isControlBall = true;
      theHeadControlMode = HeadControl::lookAtBall;
      if(ball.positionRobot.y() >= 0)
        WalkToTarget(Pose2f(0.4f, .7f, 0.7f),
                     Pose2f(0.f, ball.positionRobot.x() - kickX, ball.positionRobot.y() - kickY));//考虑能否使用pid进行对球
      if(ball.positionRobot.y()<0)
        WalkToTarget(Pose2f(0.4f, .7f, 0.7f),
                     Pose2f(0.f, ball.positionRobot.x() - kickX, ball.positionRobot.y() + kickY));
    }
  }

  state(kick)
  {
    transition
    {
      if(action_done || state_time > 1000)
        goto start;
    }
    action
    {
        behavior.supporterOutput.isRequestNotAllowed = true;
        behavior.supporterOutput.isControlBall = true;
      theHeadControlMode = HeadControl::lookForward;
      if(robot.pose.translation.x() < -1500)//当机器人在比较后方的时候，就使用小一点的力气将球踢到前场。
      {
    		  Kick(useRightFoot);
      }
      else
      {
    	  Kick(useRightFoot);
      }
    }
  }

  state(goToShootPoint)
  {
	  transition
	  {
		  if(ball.notSeenTime() > 2500)
		  {
			  goto start;
		  }
		  if(action_done) //|| (ball.positionRobot.norm() < 300 && fabs(goals.angleToGoal - ball.positionRobot.angle()) < 60_deg))
			  goto alignToGoal;
	  }
	  action
	  {
		  GoToShootPointSupporter(target);
	  }
  }
}
