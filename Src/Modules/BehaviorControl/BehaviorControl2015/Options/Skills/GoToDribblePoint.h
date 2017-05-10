option(GoToDribblePoint, (Vector2f)(Vector2f(4500.f, 0.f)) refPoint, (Vector2f)(Vector2f(80.f, 80.f)) tolerate)
{
	const float distance = 300.f;
	Vector2f shootPoint;
	if(ball.global.x() > 4000.f && fabs(ball.global.y()) < 600.f)
	{
		shootPoint.x() = ball.global.x() + (ball.global.x()
			      - refPoint.x()) * distance / (ball.global - refPoint).norm();//当机器人没有看到球，ball.global保持上一次的值
		shootPoint.y() = ball.global.y() + (ball.global.y()
			      - refPoint.y()) * distance / (ball.global - refPoint).norm();
	}
//	else
//	{
//		shootPoint = Vector2f(ball.global.x(), ball.global.y());
//	}
	Vector2f refPointRelative;
	refPointRelative = theRobotPose.inverse() * refPoint;
	Angle refAngle;
	refAngle = refPointRelative.angle();
	float distanceToLine = fabs((shootPoint.y() - robot.y) * ball.global.x() -
	      (shootPoint.x() - robot.x) * ball.global.y() + shootPoint.x() * robot.y
	      - shootPoint.y() * robot.x) / Vector2f(shootPoint.y() - robot.y, shootPoint.x() - robot.x).norm();//distanceToLine是球到机器人与射門点的连线的距离，不会超过300
	//  std::cout<<"distanceToLine--->"<<distanceToLine<<std::endl;
	Vector2f dir = ball.positionRobot - refPointRelative;
	dir.normalize();
	float length1 = -refPointRelative.x() * dir.x() -refPointRelative.y() * dir.y();
	//  float length1 = (-refPointRelative) * (ball.position - refPointRelative);
	float length2 = (ball.positionRobot - refPointRelative).norm();
	bool between = length1 > length2 + 100 ? false : true;
	//  std::cout<<"length--->"<<length1<<","<<length2<<std::endl;
	bool walkStraightToBall = true;

	common_transition
	  {

	  }

	  initial_state(judge)
	  {
	    transition
	    {
	        goto walkToShootPoint;
	    }
	    action
	    {
	      theHeadControlMode = HeadControl::lookAtBall;
	    }
	  }

	  state(walkToShootPoint)
	  {
	    transition
	    {
	      if((action_done && !walkStraightToBall) || (ball.distance < 400 && fabs(ball.angleRad) < 90_deg))//aciton_done有问题
	        goto turnaround;
	    }
	    action
	    {
	      theHeadControlMode = HeadControl::lookAtBall;
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
	    			  }
	    	if(ball.distance > 600)
	    	{
	    		WalkToTarget(Pose2f(0.3f, 0.8f, 0.5f), Pose2f(ball.angleRad, ball.positionRobot.x() - 500, 0));
	    		walkStraightToBall = true;
	    	}
	    	else
	    	{
	    		WalkToPoint(Pose2f(shootPoint.x(), shootPoint.y()),
	                  Pose2f(8_deg, tolerate), false);
	    		walkStraightToBall = false;
	    	}
	    }
	  }

	  state(turnaround)
	  {
		  transition
		  {
			  if(fabs(refAngle) < 10_deg || fabs(ball.angleDeg < 10_deg))
			  {
				  goto finish;
			  }
		  }
		  action
		  {
			  theHeadControlMode = HeadControl::lookAtBall;
			  if(ball.angleDeg > 0)
			  {
				  WalkAtSpeedPercentage(Pose2f(0.8f, 0.f, 0.f));
			  }
			  else
			  {
				  WalkAtSpeedPercentage(Pose2f(-0.8f, 0.f, 0.f));
			  }
		  }
	  }
	  target_state(finish)
	  {
	    transition
	    {
	      if(ball.distance > 400 || fabs(ball.angleRad) > 10_deg || fabs(refAngle) > 10_deg)
	        goto judge;
	    }
	    action
	    {
	      theHeadControlMode = (state_time % 4000) < 2000 ?
	          HeadControl::lookLeftAndRight : HeadControl::lookAtBall;
	      WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.f));
	    }
	  }
}
