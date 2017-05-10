/*
 * @File:   Dribble.h
 * @Author: yongqi
 * @Usage:  通过输入目标前全局坐标，可将球带到目标点为圆心，半径50cm的圆内
 * @Date:   June 18th.2016
 */

option(Dribble2, (float) x, (float) y)
{
  Angle angle2target;
  angle2target = (theRobotPose.inverse() * Vector2f(x, y)).angle();

  common_transition
  {
    if(fabs(ball.global.x() - x) < 200 && fabs(ball.global.y() - y) < 200
        && ball.distance < 200 && fabs(ball.angleRad) < 10_deg)
      goto finish;
    angle2target = (theRobotPose.inverse() * Vector2f(x, y)).angle();
    if(ball.distance > 500 || (fabs(angle2target) > 20_deg && 0) || fabs(ball.angleRad) > 20_deg)
      goto close2ball;
  }

  initial_state(close2ball)
  {
	  transition
	  {
//	     if(action_done || (ball.distance < 300.f))
//	     {std::cout<<ball.distance<<std::endl;
//	        goto goRoundBall;
//	     }
		  if(action_done) //|| (ball.positionRobot.norm() < 300 && fabs(goals.angleToGoal - ball.positionRobot.angle()) < 60_deg))
		  {
//			  if((target - ball.position).norm() < 500)
				  goto dribble;
		  }
	  }
	  action
	  {
	     theHeadControlMode = HeadControl::lookAtBall;
//	     WalkToBall(100.f,0.f);
	     GoToDribblePoint();
	  }
  }
  state(goRoundBall)
  {
	  transition
	  {
	     if(ball.distance > 500.f)
	        goto close2ball;
	     if(action_done)// || fabs(refAngle) < 5_deg)
	        goto dribble;
	  }
	  action
	  {
	     theHeadControlMode = HeadControl::lookAtBall;
	     GoBehindBall(4500.f, 0.f);
	  }
  }

  state(dribble)
  {
    transition
    {
      if(ball.distance > 500 || fabs(angle2target) > 20_deg || fabs(ball.angleRad) > 20_deg)
        goto close2ball;
//      if(fabs(angle2target) > 15_deg || fabs(ball.angleRad) > 15_deg)
//        goto adjustRotation;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
//      WalkToTarget(Pose2f(0.f, 0.6f, .2f), Pose2f(0.f, ball.x, ball.y), true);
      WalkToTarget(Pose2f(0.f, 0.4f, .2f), Pose2f(0.f, ball.x, ball.y), true);
    }
  }

  state(adjustRotation)
  {
    transition
    {
      if(ball.distance > 500 || fabs(angle2target) > 20_deg || fabs(ball.angleRad) > 20_deg)
        goto close2ball;
      if(fabs(angle2target) < 8_deg && fabs(ball.angleRad) < 8_deg)
        goto dribble;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2f(0.4f, 0.2f, .4f), Pose2f(angle2target, ball.x, ball.y));
    }
  }

  target_state(finish)
  {
    transition
    {
      if(fabs(ball.global.x() - x) > 200 || fabs(ball.global.y() - y) > 200)
        goto close2ball;
    }
    action
    {
      theHeadControlMode = (state_time % 4000) < 2000 ?
          HeadControl::lookLeftAndRight : HeadControl::lookAtBall;
      WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.f));
    }
  }

}
