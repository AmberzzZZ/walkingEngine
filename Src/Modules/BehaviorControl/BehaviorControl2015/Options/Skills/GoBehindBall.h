/*
 * File:   GoBehindBall.h
 * Author: dmx
 *add an angle in LibBall.cpp and .h
 * Created on 2014年3月11日, 下午9:02
 */

option(GoBehindBall, (float) (4500) x, (float) (0) y)
{
  const float ballDistance = 200.f;
  const float turnSpeed = 0.3f;
  const float leftSpeed = 0.6f;

  float angleToShoot = (theRobotPose.inverse() * Vector2f(x, y)).angle();

  common_transition
  {

  }

  initial_state(start)
  {
    const Pose2f speed(0.5f, 0.f, 0.f);
    transition
    {
      if(fabs(ball.angleRad) < 5_deg)
      {
        if(common.between(ball.distance, ballDistance - 50.f, ballDistance + 50.f))
        {
          if(fabs(angleToShoot) > 5_deg)
            goto turnAround;
          else
            goto kickAlign;
        }
        else
        {
          goto closeToBall;
        }
      }
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      if(ball.angleRad > 0)
        WalkAtSpeedPercentage(speed);
      else
        WalkAtSpeedPercentage(-speed);
    }
  }

  state(closeToBall)
  {
    transition
    {
      if(fabs(ball.angleRad) < 25_deg)
      {
        if(common.between(ball.distance, ballDistance - 50.f, ballDistance + 50.f))
        {
          if(fabs(angleToShoot) > 5_deg)
            goto turnAround;
          else
            goto kickAlign;
        }
      }
      else
      {
        goto start;
      }
    }
    action
    {
    	theHeadControlMode =HeadControl::lookAtBall;
      WalkToBall(ballDistance);
    }
  }

  state(turnAround)
  {
    transition
    {
      //      if(ball.distance > 2 * ballDistance)
        //        goto start;
      if(fabs(angleToShoot) < 5_deg)
        goto kickAlign;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      float x_speed = 0;
      if(fabs(ball.positionRobot.x()) < 100)
      {
    	  x_speed = -0.15;
      }
      else if(fabs(ball.positionRobot.x()) > 200)
      {
    	  x_speed = 0.15;
      }
      else
      {
    	  x_speed = 0;
      }//当机器人离球太近或者太远的时候稍微调整一下位置。
//      std::cout<<"x_speed--->"<<x_speed<<std::endl;
      if(angleToShoot >= 0.f)
      {
        WalkAtSpeedPercentage(Pose2f(turnSpeed, x_speed, -leftSpeed));
      }
      else
      {
        WalkAtSpeedPercentage(Pose2f(-turnSpeed, x_speed, leftSpeed));//绕球
      }
    }
  }

  target_state(kickAlign)
  {
    transition
    {
      if(fabs(ball.distance - ballDistance) > 80.f || fabs(angleToShoot) > 10_deg || fabs(ball.angleRad) > 10_deg)
        goto start;
    }
    action
    {
              PlaySound("allright.wav");
      theHeadControlMode = HeadControl::lookAtBall;
      WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.f));//??
    }
  }
}
