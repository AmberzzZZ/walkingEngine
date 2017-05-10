option(GoToShootPointWithPath, (Vector2f)(Vector2f(4500.f, 0.f)) refPoint, (Vector2f)(Vector2f(80.f, 80.f)) tolerate)//减少goaroundball状态的次数
{
  const float distance = 210.f;
  const float Re = 100.f; //机器人弧线的半径 先设置为300   可修改
  static int cont = 0;
  Vector2f shootPoint;
  shootPoint.x() = ball.positionField.x() + (ball.positionField.x()
      - refPoint.x()) * distance / (ball.positionField - refPoint).norm();//当机器人没有看到球，ball.global保持上一次的值
  shootPoint.y() = ball.positionField.y() + (ball.positionField.y()
      - refPoint.y()) * distance / (ball.positionField - refPoint).norm();

//  Vector2f turnPoint;
//  turnPoint.x() = ball.global.x() - (ball.global.y()
//	  - refPoint.y()) * ( distance + 100.f ) / (ball.global - refPoint).norm();
//  turnPoint.y() = ball.global.y() + (ball.global.x()
//	  - refPoint.x()) * ( distance + 100.f ) / (ball.global - refPoint).norm();

//  Vector2f turnPoint2;
//    turnPoint2.x() = ball.global.x() + (ball.global.y()
//  	  - refPoint.y()) * ( distance + 100.f ) / (ball.global - refPoint).norm();
//    turnPoint2.y() = ball.global.y() - (ball.global.x()
//  	  - refPoint.x()) * ( distance + 100.f ) / (ball.global - refPoint).norm();

//  Vector2f Circle;   //定义圆心坐标
//  Vector2f Answer1;  //定义切点的第一个解
//  Vector2f Answer2;  //定义切点的第二个解
//  Vector2f ChangePoint;

//  Vector2f CircleL; //定义左边的圆
//  CircleL.x() = shootPoint.x() - Re * fabs(ball.global.y()
//	      - refPoint.y()) / (ball.global - refPoint).norm();
//  CircleL.y() = shootPoint.y() + Re * fabs(ball.global.x()
//	      - refPoint.x()) / (ball.global - refPoint).norm();

//  Vector2f CircleR; //定义右边的圆
//  CircleR.x() = shootPoint.x() + Re * fabs(ball.global.y()
//	      - refPoint.y()) / (ball.global - refPoint).norm();
//  CircleR.y() = shootPoint.y() - Re * fabs(ball.global.x()
//  	      - refPoint.x()) / (ball.global - refPoint).norm();

//  float length_ShootPoint = (robot.pose.translation - shootPoint).norm();   //定义到击球点的距离
//  float length_Circle;  // 定义到圆心的距离
//  float length_ChangePoint;   // 定义到切点的距离
//  float length_answer1;  //第一个解到球的距离
//  float length_answer2;  //第二个解到球的距离

  Vector2f refPointRelative;
  refPointRelative = theRobotPose.inverse() * refPoint;
//  Vector2f shootPointRobot = theRobotPose.inverse() * shootPoint;
//  Vector2f turnPointRobot = theRobotPose.inverse() * turnPoint;
//  Vector2f turnPointRobot2 = theRobotPose.inverse() * turnPoint2;
//  Angle ShootPointAngle;
//  ShootPointAngle = shootPointRobot.angle();

//  static Vector2f CircleRelative;

  Angle refAngle;
  refAngle = refPointRelative.angle(); //机器人坐标系 球门的角度
//  static Angle CirAngle;     //圆心在机器人坐标中的角度
//  float limitAngle;

  /*
  float distanceToLine = fabs((shootPoint.y() - robot.y) * ball.global.x() -
      (shootPoint.x() - robot.x) * ball.global.y() + shootPoint.x() * robot.y
      - shootPoint.y() * robot.x) / Vector2f(shootPoint.y() - robot.y, shootPoint.x() - robot.x).norm();//distanceToLine是球到机器人与射門点的连线的距离，不会超过300
 */

////  std::cout<<"distanceToLine--->"<<distanceToLine<<std::endl;
//  Vector2f dir = ball.position - refPointRelative;
//  dir.normalize();
//  float length1 = -refPointRelative.x() * dir.x() -refPointRelative.y() * dir.y();
////  float length1 = (-refPointRelative) * (ball.position - refPointRelative);
//  float length2 = (ball.position - refPointRelative).norm();
//  bool between = length1 > length2 + 100 ? false : true;
////  std::cout<<"length--->"<<length1<<","<<length2<<std::endl;

//  bool walkStraightToBall = true;
//  static bool turnRightFlag ;    //  定义一个转向标志位

  //float offset = (-0.067*ball.x + 80)/180*3.1415;

  bool useRightFoot;  //增加一个踢球动作

  if(ball.positionRobot.y() < 0.f)
      useRightFoot = true;
    else
      useRightFoot = false;


  if(theRole.role == Role::striker && ball.distance < 500 && fabs(ball.angleRad) < 90_deg)
  {
    behavior.strikerOutput.isRequestNotAllowed = true;
    behavior.strikerOutput.isControlBall = true;
  }
//	std::cout<<"alignFuzzy.rhoValue-------->"<<alignFuzzy.rhoValue<<std::endl;
//	std::cout<<"alignFuzzy.phiValue-------->"<<alignFuzzy.phiValue<<std::endl;
//	std::cout<<"alignFuzzy.gammaValue------>"<<alignFuzzy.gammaValue<<std::endl;
//	std::cout<<"ball-obstacle distance----->"<<sqrt(sqr (ball.positionRobot.x() - obstacle.x) + sqr (ball.positionRobot.y() - obstacle.y))<<std::endl;
//	std::cout<<"ball.x - obstacle.x-------->"<<ball.positionRobot.x() - obstacle.x<<std::endl;
//	std::cout<<"obstacle.x----------------->"<<ball.positionRobot.x()<<std::endl;

  common_transition
  {

  }

  initial_state(judge)
  {
    transition
    {

        if(fabs(ball.angleRad) > 10_deg)	//@xqc
        {

        }
        else
            goto walkToShootPoint;
//    	     goto turnToBall;

//        if(refAngle >= 0)
//            turnRightFlag = false;
//        else
//            turnRightFlag = true;

//        if(fabs(refAngle) < 30_deg && fabs(fabs(refAngle) - fabs(ball.angleRad)) < 20_deg )
//             goto walkToPointDirectly;
			//else
    		//goto dribble;

    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      TurnToBall(5.f, 0.5f);
    }
  }


  state(walkToShootPoint)
  {
    transition
    {

//    	if(ball.distance > 600.f && fabs(ball.angleRad) > 10_deg)
//    		goto turnToBall;

//    	if(alignFuzzy.rhoValue < 100.f && fabs(alignFuzzy.phiValue) < (Angle)20_deg && fabs(alignFuzzy.gammaValue) < (Angle)20_deg)
//    			//alignFuzzy.useLeftFoot  true--left   flase--right	//@xqc:origin_10_deg
//    		goto kick;

//        if(!ball.isInOppPenalty() && ball.distance < 900 && obstacle.x < 500 && obstacle.x > 5 && fabs(obstacle.y) < 250 && ball.x - obstacle.x > 100)	//@xqc:add
//        		goto avoidObstacle;

//    	if(ball.distance > 450.f && fabs(refAngle - ball.angleRad) < 20_deg && fabs(refAngle) < 30_deg)	//@xqc
//    		goto walkToPointDirectly;

//    	if(ball.distance < 400.f && fabs(ball.angleRad) < 35_deg )
//    			goto goRoundBall;	@xqc



//    	if(fabs(refAngle) > 100_deg)
//    	{
//    		if(action_done && !walkStraightToBall || fabs(ball.angleRad) > limitAngle && ball.distance < 500.f)
//    	            goto turnaround;
////    		if(action_done && !walkStraightToBall || ball.distance < 430.f )
////    			  goto turnToShootPoint;
//
//    	}
//    	else
//    	{
//    		if(action_done && !walkStraightToBall || fabs(ball.angleRad) > 50_deg && ball.distance < 450.f)
//    		    	goto turnaround;
////    			goto turnToBall;
//    	}

    		//goto turnToBall;

    }
    action
    {

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

//      if(ball.distance > 900)
//      {
//            //WalkToTargetWithPath(Pose2f(0.3f, 0.8f, 0.5f), Pose2f(ball.angleRad, ball.x - 500, 0));	//@xqc
//    	    WalkToTargetWithPath(Pose2f(0.3f, 0.7f, 0.3f), Pose2f(ball.angleRad, shootPoint.x(), shootPoint.y()));
//
//      }
//      else
      {
//		  AlignMent();	//@xqc
    	  GoToBallAndKickStriker(refPoint);
      }

    }
  }

  state(searchForBall)
  {
    transition
    {
      if(ball.notSeenTime() < 300)
        goto judge;
    }
    action
    {
//      theHeadControlMode = state_time % 6000 < 3000? HeadControl::lookLeftAndRightDown : HeadControl::lookLeftAndRight;
    	// theHeadControlMode = state_time % 6000 < 3000? HeadControl::lookDown : HeadControl::lookForward;
     //  WalkAtSpeedPercentage(Pose2f(0.8f, 0.f, 0.f));
      SearchForBallFast();
    }
  }

    state(avoidObstacle)	//@xqc:add state
    {
    	transition
		{
    		if(action_done)
    			goto judge;
		}
    	action
		{
    		theHeadControlMode = HeadControl::lookLeftAndRight;
    		AvoidObstacle(ball.global);

    	    alignFuzzy.rhoValue = 9000.f;	//@xqc:add
    	    alignFuzzy.phiValue = (Angle)180_deg;	//@xqc:add
    		alignFuzzy.gammaValue = (Angle)180_deg;	//@xqc:add
		}
    }

	state(kick)
	{
		transition
		{
//    	if (pass)
//    	    	    		goto turnPass;
			if(action_done || state_time > 3000)
				goto finish;
		}
		action
		{
			behavior.strikerOutput.isRequestNotAllowed = true;
			behavior.strikerOutput.isControlBall = true;
			theHeadControlMode = HeadControl::lookForward;
			if(robot.pose.translation.x() < -1500) //当机器人在比较后方的时候，就使用小一点的力气将球踢到前场。
			{
				Kick(useRightFoot);
			}
			else
			{
				Kick(useRightFoot);
			}
		}
	}


	/*
	  state(dribble)
	    {
	        transition
	        {
	           if(action_done || ball.positionRobot.norm() > 600 || fabs(ball.positionRobot.angle()) > 90_deg)
	             goto start;
	           else if(ball.notSeenTime() > 4000)
	             goto searchForBall;
	        }
	        action
	        {
	            Dribble(target.x(), target.y());
	        }
	    }
	*/

//	  state(turnToBall)
//	  {
//	      transition
//	      {
//	    	  /*
//	        if(action_done || fabs(ball.angleRad) < 5_deg || fabs(refAngle) > 150_deg)
//	          goto walkToShootPoint;
//	          */
//	    	  if(action_done)
//	    		  {
//	    		           goto judge;
//	    		  }
//	      }
//	      action
//	      {
//	    	 // if(fabs(refAngle) < 150_deg)
//	             TurnToBall(5.f, 0.5f);
//	      }
//	  }


//	  state(goRoundBall)
//	  {
//	    transition
//	    {
//	      if(ball.distance > 350.f) //将500修改为350
//	        //goto judge;
//	    	goto walkToShootPoint;
//	      if( fabs (ball.angleRad - refAngle) < 5_deg)
//	        goto finish;
//	    }
//	    action
//	    {
//	      theHeadControlMode = HeadControl::lookAtBall;
//	      GoBehindBall(refPoint.x(), refPoint.y());
//	    }
//	  }



//	state(turnaround)
//	{
//		  transition
//		  {
//
//			  //CircleRelative = theRobotPose.inverse() * Circle;
//			 // CirAngle = CircleRelative.angle();
//			  //if(fabs(fabs(CirAngle) - 90_deg) > 5_deg)
//			   //  goto turnToShootPoint;
//
//			  //if(fabs(refAngle) < 20_deg || fabs(ball.angleDeg) < 20)
//			  if( fabs(refAngle - ball.angleRad) < 10_deg && fabs(refAngle) < 10_deg)
//			  {
//				  goto finish;
//			  }
//			  if(ball.distance > 500)       //增加一个判断的转换
//			  {
//				  goto judge;
//			  }
//		  }
//		  action
//		  {
//
//			  theHeadControlMode = HeadControl::lookAtBall;
//			  if(refAngle - ball.angleRad > 0)
//			  {
//
//				  //WalkAtSpeedPercentage(Pose2f(0.8f, 0.f, 0.f));
//	//			  WalkAtSpeedPercentage(Pose2f(0.8f, 0.4f, 0.f));  //增加了前进速度
//	//			  WalkAtSpeedPercentage(Pose2f(0.8f, 0.f, -0.5f));
//				  WalkToTarget(Pose2f(.8f, 0.4f, 0.5f),
//				                       Pose2f(refAngle, shootPointRobot.x(), shootPointRobot.y()));
//			  }
//			  else
//			  {
//				  //WalkAtSpeedPercentage(Pose2f(-0.8f, 0.f, 0.f));
//	//			  WalkAtSpeedPercentage(Pose2f(-0.8f, 0.4f, 0.f)); //增加了前进速度
//	//			  WalkAtSpeedPercentage(Pose2f(-0.8f, 0.f, 0.5f));
//				  WalkToTarget(Pose2f(.8f, 0.4f, 0.5f),
//				 			           Pose2f(refAngle, shootPointRobot.x(), shootPointRobot.y()));
//			  }
//
//			  //if(fabs(refAngle) > 60_deg && ball.distance < 200.f)
//			  //	  Kick(useRightFoot,0.1);       //使用0.4的力度踢球
//		  }
//	}

//	state(turnToShootPoint)
//	{
//		  transition
//		  {
//	         if(fabs( ball.angleRad ) < 5_deg && (robot.pose.translation - turnPoint).norm() < 70.f)
//	      	   goto turnaround;
//
//	         if(fabs( ball.angleRad ) < 5_deg && (robot.pose.translation - turnPoint2).norm() < 70.f)
//	             goto turnaround;
//
//	         if(ball.distance > 500)
//	      	   goto judge;
//		  }
//		  action
//		  {
//			  theHeadControlMode = HeadControl::lookAtBall;
//			  if(turnRightFlag)
//				  WalkToTarget(Pose2f(.3f, 0.1f, 0.5f), Pose2f( ball.angleRad, turnPointRobot.x(), turnPointRobot.y()));
//			  else
//				  WalkToTarget(Pose2f(.3f, 0.1f, 0.5f), Pose2f( ball.angleRad, turnPointRobot2.x(), turnPointRobot2.y()));
//		  }
//	}


//	  state(walkToPointDirectly)
//	  {
//		transition
//		{
//	//		if(fabs(ball.angleRad) > 10_deg && ball.distance > 500.f)	//@xqc
//	//			goto turnToBall;
//	//	    if (ball.distance < 190.f && fabs(ball.angleRad) < (Angle)10_deg && fabs(refAngle) < (Angle)10_deg)
//	//	    	goto kick;
//	    	if(alignFuzzy.rhoValue < 100.f && fabs(alignFuzzy.phiValue) < (Angle)20_deg && fabs(alignFuzzy.gammaValue) < (Angle)20_deg)
//	    		//alignFuzzy.useLeftFoot  true--left   flase--right	//@xqc:origin_10_deg
//	    		goto kick;
//
//	        if(!ball.isInOppPenalty() && obstacle.x < 500 && obstacle.x > 5 && fabs(obstacle.y) < 250 && ball.x - obstacle.x > 100)	//@xqc:add
//	        		goto avoidObstacle;
//
//			if(fabs(refAngle) > 30_deg)
//				goto judge;
//
//	//        if(action_done && ball.distance < 300.f)
//	//        	 goto finish;
//
//		}
//		action
//		{
//			theHeadControlMode = HeadControl::lookAtBall;
//			if(ball.distance > 450)
//			{
//	//		    WalkToTargetWithPath(Pose2f(0.3f, 0.8f, 0.5f), Pose2f(ball.angleRad, ball.x - 180, 0));	//@xqc
//				WalkToTargetWithPath(Pose2f(0.3f, 0.8f, 0.5f), Pose2f(ball.angleRad, shootPoint.x(), shootPoint.y()));
//			    //walkStraightToBall = true;
//			}
//			else
//			{
//	//		    WalkToPoint(Pose2f(shootPoint.x(), shootPoint.y()),	Pose2f(8_deg, tolerate), false);	//@xqc
//				AlignMent();
//			    //walkStraightToBall = false;
//			}
//		}
//	  }


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

      alignFuzzy.rhoValue = 9000.f;	//@xqc
      alignFuzzy.phiValue = (Angle)180_deg;	//@xqc
	  alignFuzzy.gammaValue = (Angle)180_deg;	//@xqc
    }
  }

}
