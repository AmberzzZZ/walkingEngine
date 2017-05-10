/**
 * @file WalkToBall.h
 * @author Jimin Wang
 * @date Jun 18, 2014
 * @Usage TurnToBall(); or TurnToBall(10.f, 0.8f)
 * @param distance is the expected distance between ball and robot
 *
 */

option(WalkToBall, (float)(200.f) x, (float)(0.f) y)
{
  const float tolerateX = 50.f;
  const float tolerateY = 50.f;
  const float farAwayDistance = 500.f;
  const float farAwayDeg = 30.f;
  const float nearbyDeg = 45.f;
  const float turnToBallDeg = 10.f;

  float obsWidth = 250.0f;
//  if(ball.distance<1000)
//	  obsWidth = 500.f;
//  if(fabs(ball.x-obstacle.x)<400)
//	  obsWidth = 300.f;

  Vector2f globalObstacle = Geometry::relative2FieldCoord(Pose2f(robot.rotation,robot.x,robot.y),obstacle.x,obstacle.y);
  Vector2f ballToRobot = robot.pose.translation-ball.global;
  Vector2f ballToObstacle = globalObstacle-ball.global;
  float angleDiff = fabs(ballToRobot.angle()-ballToObstacle.angle());
  common_transition
  {
      if(fabs(ball.positionRobot.x() - x) < tolerateX && fabs(ball.positionRobot.y() - y) < tolerateY)
        goto finished;

      if (!(ball.isInOppPenalty() || (sqr (ball.positionRobot.x() - obstacle.x) + sqr (ball.positionRobot.y() - obstacle.y)) < sqr (400)))//any condition lead not to avoid球到障碍物的距离计算错误
      {
      	if (obstacle.x < 500 && obstacle.x > 5 && fabs(obstacle.y) < obsWidth && (ball.positionRobot.x() - obstacle.x) > 100)	//@>100?
      	{
      		goto avoidObstacle;//original 100
      	}
      	if (obstacle.x >= 500 && obstacle.x <= 1200 && angleDiff<1.0 && fabs(obstacle.y) < obsWidth
      			/*&& (ball.global -Geometry::relative2FieldCoord (Pose2f (robot.rotation, robot.x, robot.y), obstacle.x, obstacle.y)).norm () > 800*/
  				&& ball.positionRobot.x() - obstacle.x > 50)//@hjq original.norm()>800
      	{
      		goto avoidPath;
      	}
      }
  }

//  common_transition
//  {
//    if(fabs(ball.x - x) < tolerateX && fabs(ball.y - y) < tolerateY)
//      goto finished;
//
//    if (!(ball.isInOppPenalty() || (sqr (ball.x - obstacle.x) + sqr (ball.y - obstacle.y)) < sqr (400)))//any condition lead not to avoid
//    {
//    	if (obstacle.x < 500 && obstacle.x > 5 && fabs(obstacle.y) < obsWidth && ball.x - obstacle.x > 100)	//@>100?
//    	{
//    		goto avoidObstacle;//original 100
//    	}
//    	if (obstacle.x >= 500 && obstacle.x <= 1200 && fabs (obstacle.y) < obsWidth
//    			/*&& (ball.global -Geometry::relative2FieldCoord (Pose2f (robot.rotation, robot.x, robot.y), obstacle.x, obstacle.y)).norm () > 800*/
//				&& ball.x - obstacle.x > 50)//@hjq original.norm()>800
//    	{
//    		goto avoidPath;
//    	}
//    }
//  }

  initial_state(judge)
  {
    transition
    {
      if ((ball.distance > farAwayDistance && fabs(ball.angleDeg) > turnToBallDeg)
          || (ball.distance <= farAwayDistance && fabs(ball.angleDeg) > nearbyDeg))
        goto turnToBall;
      else
        goto walkStraightToBall;
    }
  }

  state(turnToBall)
  {
    transition
    {
      if(action_done)
        goto walkStraightToBall;
    }
    action
    {
      TurnToBall(5.f, 0.5f);
    }
  }

  state(walkStraightToBall)
  {
    transition
    {
      if(fabs(ball.positionRobot.x() - x) < tolerateX && fabs(ball.positionRobot.y() - y) < tolerateY)
        goto finished;
      if((ball.distance > farAwayDistance && fabs(ball.angleDeg) > farAwayDeg)
          || (ball.distance <= farAwayDistance && fabs(ball.angleDeg) > nearbyDeg))
        goto turnToBall;
    }
    action
    {
//      theHeadControlMode = (state_time % 6000) < 3000 ? HeadControl::lookAtBall :
//          HeadControl::lookLeftAndRight;
    	if(ball.distance > 2000)
    	    	{
    	    		theHeadControlMode = HeadControl::lookLeftAndRight;
    	    	}
    	    	else if(ball.distance > 800)
    	    	{
    	    	    theHeadControlMode = (state_time % 4000) < 2000 ? HeadControl::lookAtBall :
    	    	    		    	    	    	          HeadControl::lookLeftAndRight;
    	    	}
    	    	else
    	    	{
    	    	    theHeadControlMode =HeadControl::lookAtBall;
    	//    		theHeadControlMode = (state_time%6000<3000) ?
    	//    		          HeadControl::lookAtBall : HeadControl::lookLeftAndRight;
    	    	}
      if(ball.distance > farAwayDistance)
        WalkToTarget(Pose2f(0.1f, 0.8f, 0.f), Pose2f(ball.angleRad, ball.positionRobot.x() - x, 0));
      else
        WalkToTarget(Pose2f(0.f, 0.5f, .2f), Pose2f(0.f, ball.positionRobot.x() - x, ball.positionRobot.y() - y));
    }
  }


  state(avoidObstacle)
  {
	  transition
	  {
		  if (action_done)
		  {
//			  std::cout<<"avoidObastacle_done--------------------"<<std::endl;
	      	  goto judge;
		  }
	  }
	  action
	  {
		  theHeadControlMode = HeadControl::lookLeftAndRight;
	      AvoidObstacle(ball.global);
	  }
  }
  state(avoidPath)
  {
	  transition
      {
        if (action_done)
        {
//		  std::cout<<"avoidPath_done--------------------"<<std::endl;
      	  goto judge;
        }
      }
      action
      {
      	theHeadControlMode = HeadControl::lookLeftAndRight;
      	AvoidPath(Pose2f(ball.angleRad,ball.global.x(),ball.global.y()));
      }
  }



  target_state(finished)
  {
    transition
    {
      if(fabs(ball.positionRobot.x() - x) > tolerateX || fabs(ball.positionRobot.y() - y) > tolerateY)
        goto judge;
    }
    action
    {
      theHeadControlMode = (state_time % 6000) < 3000 ? HeadControl::lookAtBall :
          HeadControl::lookLeftAndRight;
      WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.f));
    }
  }
}
