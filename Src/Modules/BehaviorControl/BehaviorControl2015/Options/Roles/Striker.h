option(Striker) //把树的广度和深度都减小
{
	const float sideKickKickX = 150.f;
	const float sideKickKickY = 30.f;
	static bool isKickOffLimit = false;
	static bool shootOrNot = true;
	static bool passByOrNot = false;
	static bool passByLeft = true;
	static bool lastStateIsPassBy = false;
	static Vector2f target(4500.f, 0.f);
	static Vector2f PassByTarget(4500.f, 0.f);
	static float lastPassDribbleY = Geometry::relative2FieldCoord(theRobotPose, obstacle.center).y() + 300.f;
	int shootType = 0;
	Vector2f opponentNearestOnField;
	int interSectionPoint = striker.getIntersection(ball.positionField,target,opponentNearestOnField);
	Vector2f opponentNearestOnRobot = (theRobotPose.inverse() * opponentNearestOnField);
//	std::cout<<"Robot.rotation!!!!!!!!!!!!!!!!!!!!!!!"<<robot.rotation * 180.f /3.1415926<<std::endl;
	if(interSectionPoint == 2 && ball.positionField.x() < 3200.f && (ball.isTeamPositionValid || ball.wasSeen()) && ball.positionRobot.norm() < 400.f  && (opponentNearestOnField.x() - robot.x) > 0.f && striker.numObstacleNearby > 0 && ball.positionField.x() > robot.x)
		passByOrNot = true;
	else
		passByOrNot = false;

//	std::cout<<"passByLeft!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!----------------!!::::: "<<passByLeft<<std::endl;
//	std::cout<<"opponentNearestOnRobot.angle!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!::"<<fabs(opponentNearestOnRobot.angle()) * 180.f /3.1415926f<<std::endl;
//	std::cout<<"opponentNearestOnField.x!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!::"<<opponentNearestOnField.x()<<std::endl;
//	std::cout<<"opponentNearestOnField.y!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!::"<<opponentNearestOnField.y()<<std::endl;
//	std::cout<<"opponentNearestOnRobot.x!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!::"<<opponentNearestOnRobot.x()<<std::endl;
//	std::cout<<"opponentNearestOnRobot.y!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!::"<<opponentNearestOnRobot.y()<<std::endl;
//	std::cout<<"opponentNearestOnRobot.x!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!::"<<(theRobotPose.inverse() * opponentNearestOnField).x()<<std::endl;
//	std::cout<<"opponentNearestOnRobot.y!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!::"<<(theRobotPose.inverse() * opponentNearestOnField).y()<<std::endl;
//	std::cout<<"obstacle.center.x!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!::"<<obstacle.center.x()<<std::endl;
//	std::cout<<"obstacle.center.y!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!::"<<obstacle.center.y()<<std::endl;


	common_transition
	{
		Vector2f goalCenter(4500.f,0.f);
		Vector2f goalLeft(4500.f,750.f);
		Vector2f goalRight(4500.f,-750.f);
		Vector2f goalLeftLine(4500.f,250.f);
		Vector2f goalRightLine(4500.f,-250.f);

		Vector2f drawTarget = theRobotPose.inverse() * target;
		Vector2f drawpassByTarget = theRobotPose.inverse() * PassByTarget;
		//CIRCLE("module:Behavior2015:shootPath", target.x(), target.y(), 50.f,
        //     10, Drawings::solidPen, ColorRGBA::yellow, Drawings::noPen, ColorRGBA());
		LINE("module:Behavior2015:shootPath", ball.positionRobot.x(), ball.positionRobot.y(), (theRobotPose.inverse() * goalLeft).x(), (theRobotPose.inverse() * goalLeft).y(),
           10, Drawings::solidPen, ColorRGBA::blue);
		LINE("module:Behavior2015:shootPath", ball.positionRobot.x(), ball.positionRobot.y(), (theRobotPose.inverse() * goalLeftLine).x(), (theRobotPose.inverse() * goalLeftLine).y(),
           10, Drawings::solidPen, ColorRGBA::blue);
		LINE("module:Behavior2015:shootPath", ball.positionRobot.x(), ball.positionRobot.y(), (theRobotPose.inverse() * goalRightLine).x(), (theRobotPose.inverse() * goalRightLine).y(),
           10, Drawings::solidPen, ColorRGBA::blue);
		LINE("module:Behavior2015:shootPath", ball.positionRobot.x(), ball.positionRobot.y(), (theRobotPose.inverse() * goalRight).x(), (theRobotPose.inverse() * goalRight).y(),
           10, Drawings::solidPen, ColorRGBA::blue);
//        std::cout<<"targetXrelative::"<<drawTarget.x()<<' '<<"targetYrelative::"<<drawTarget.y()<<std::endl;
		ARROW("module:Behavior2015:shootPath", 0, 0, drawTarget.x(), drawTarget.y(), 20, 10, ColorRGBA::orange);
		ARROW("module:Behavior2015:shootPath", 0, 0, drawpassByTarget.x(), drawpassByTarget.y(), 20, 10, ColorRGBA::blue);
		ARROW("module:Behavior2015:shootPath", 0, 0, 500.f, 0.f, 20, 10, ColorRGBA::yellow);
		// CIRCLE("module:Behavior2015:shootPath", ball.positionRobot.x(), ball.positionRobot.y(), 200.f,
  //       10, Drawings::solidPen, ColorRGBA::yellow, Drawings::noPen, ColorRGBA());


		if(!ball.isTeamPositionValid && ball.notSeenTime() > 2000)
		{
			PlaySound("lost.wav");
			goto searchBall;
		}

		if(common.isKickOff)
		{
			PlaySound("kickingOut.wav");
			goto kickOffBehavior;
		}

	}

	initial_state(start)
	{
		transition
		{
			// if(ball.positionRobot.norm() > 1200.f)
			// 	goto walkToBall;
		    if(interSectionPoint == 2 && (ball.positionField.x() < 3200.f || fabs(ball.positionField.y()) > 1100.f) && (ball.isTeamPositionValid || ball.wasSeen()) && ball.positionRobot.norm() < 350.f  && (opponentNearestOnField.x() - robot.x) > 0.f && striker.numObstacleNearby > 0 && ball.positionField.x() > robot.x && ball.positionRobot.x() > 100.f ) //&& obstacle.center.norm() < 500.f&& (opponentNearestOnField.x() - ball.positionField.x()) > 50.f
		    {
		        PassByTarget.x() = opponentNearestOnField.x() + 100.f;
		        // if(ball.positionField.y() - opponentNearestOnField.y() > 100.f)
		        // 	target.y() = opponentNearestOnField.y() + 300.f;
		        // else if(ball.positionField.y() - opponentNearestOnField.y() < -100.f)
		        // 	target.y() = opponentNearestOnField.y() - 300.f;
		        float passByOffset = 400.f;
		        if(common.between(opponentNearestOnField.y(),ball.positionField.y()-200.f,ball.positionField.y()+200.f) && fabs(opponentNearestOnRobot.angle()) < 20_deg)
		        	passByOffset = (opponentNearestOnField.y() > ball.positionField.y())>0?-400.f:400.f;
		        else if(opponentNearestOnRobot.y() > 0.f)
		        	passByOffset = (opponentNearestOnField.y() - 400.f) < -2000.f ? 400.f:-400.f;
		        else
		        	passByOffset = (opponentNearestOnField.y() + 400.f) >  2000.f ? -400.f:400.f;

		        PassByTarget.y() = opponentNearestOnField.y() + passByOffset;
		        passByLeft = passByOffset>0? true:false;

//		        std::cout<<"passByLeft!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!:::::"<<passByLeft<<std::endl;
//		        std::cout<<"passByOffset!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!:::::"<<passByOffset<<std::endl;
		        if(!lastStateIsPassBy)
		        	PlaySound("doh.wav");
		        goto passBy;
		    }
			if(!shootOrNot && ball.positionRobot.norm() < 600.f)//			else
			{
				PlaySound("dribble.wav");
				goto dribble;
			}
			lastStateIsPassBy = false;
		}
		action
		{

			if( ball.positionRobot.norm() < 750.f && ball.positionField.y() > -750.f && ball.positionField.y() < 750.f && ball.positionField.x() > 4000 && (ball.wasSeen() || ball.isTeamPositionValid ))
			{
				Dribble(5000.f, ball.positionField.y());
			}
			else
			{
				if(ball.positionField.x() > 0.f && ball.positionRobot.norm() > 350.f && ball.positionRobot.norm() < 1000.f)
				{
					shootOrNot = striker.getStrikerKickAngle(shootType, ball.positionField,target);
//					std::cout<<"KickJudging!!"<<shootType<<std::endl;
				}
				else if(ball.positionRobot.norm() > 1200.f)
				{
					target.x() = 4500.f;
					target.y() = 0.f;
				}
//				std::cout<<"shootOrNot"<<shootOrNot<<std::endl;
//				std::cout<<"target.x() "<<target.x()<<" "<<"target.y() "<<target.y()<<std::endl;
				if(target.y() < -750.f && target.y() < ball.positionField.y())
					target.y() = 0.f;
				else if (target.y() > 750.f && target.y() > ball.positionField.y())
					target.y() = 0.f;
				LINE("module:Behavior2015:shootPath", ball.positionRobot.x(), ball.positionRobot.y(), (theRobotPose.inverse() * target).x(), (theRobotPose.inverse() * target).y(),
                10, Drawings::solidPen, ColorRGBA::yellow);
				GoToBallAndKickWithPath(target);
			}

		}
	}

	state(walkToBall)
	{
		transition
		{
			if(ball.positionRobot.norm() < 1000.f)
				goto start;
		}
		action
		{ 
			WalkToBall();
		}
	}

	state(dribble)
	{
		transition
		{
			if(interSectionPoint == 2 && (ball.positionField.x() < 3200.f || fabs(ball.positionField.y()) > 1100.f) && (ball.isTeamPositionValid || ball.wasSeen()) && ball.positionRobot.norm() < 350.f  && (opponentNearestOnField.x() - robot.x) > 0.f && striker.numObstacleNearby > 0 && ball.positionField.x() > robot.x && ball.positionRobot.x() > 100.f && ball.positionRobot.x() < opponentNearestOnRobot.x()) //&& obstacle.center.norm() < 500.f&& (opponentNearestOnField.x() - ball.positionField.x()) > 50.f
		    {
		        PassByTarget.x() = opponentNearestOnField.x() + 100.f;
		        // if(ball.positionField.y() - opponentNearestOnField.y() > 100.f)
		        // 	target.y() = opponentNearestOnField.y() + 300.f;
		        // else if(ball.positionField.y() - opponentNearestOnField.y() < -100.f)
		        // 	target.y() = opponentNearestOnField.y() - 300.f;
		        float passByOffset = 400.f;
		        // if (fabs(opponentNearestOnRobot.angle()) < 10_deg )
		        // {
		        // 	passByOffset = (opponentNearestOnField.y() > ball.positionField.y())? -400.f:400.f;
		        // }
		        if(common.between(opponentNearestOnField.y(),ball.positionField.y()-200.f,ball.positionField.y()+200.f) && fabs(opponentNearestOnRobot.angle()) < 20_deg)
		        	passByOffset = (opponentNearestOnField.y() > ball.positionField.y())>0?-400.f:400.f;
				else if(opponentNearestOnRobot.y() > 0.f)
		        	passByOffset = (opponentNearestOnField.y() - 400.f) < -2000.f ? 400.f:-400.f;
		        else
		        	passByOffset = (opponentNearestOnField.y() + 400.f) >  2000.f ? -400.f:400.f;

		        PassByTarget.y() = opponentNearestOnField.y() + passByOffset;
		        passByLeft = passByOffset>0? true:false;

//		        std::cout<<"passByLeft!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!:::::"<<passByLeft<<std::endl;
//		        std::cout<<"passByOffset!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!:::::"<<passByOffset<<std::endl;
		        if(!lastStateIsPassBy)
		        	PlaySound("doh.wav");
		        goto passBy;
		    }
			if(action_done || shootOrNot == true || ball.positionRobot.norm() > 700.f)
			{
				shootOrNot = true;
				target.x() = 4500.f;
				target.y() = 0.f;
				goto start;
			}
	        lastStateIsPassBy = false;
		}
		
		action
		{
			shootOrNot = striker.getStrikerKickAngle(shootType, ball.positionField,target);
//			std::cout<<"Dribbletarget.x() "<<target.x()<<" "<<"Dribbletarget.y() "<<target.y()<<std::endl;
			LINE("module:Behavior2015:shootPath", ball.positionRobot.x(), ball.positionRobot.y(), (theRobotPose.inverse() * target).x(), (theRobotPose.inverse() * target).y(),
                10, Drawings::solidPen, ColorRGBA::yellow);
			BizhangDribble(target.x(), target.y());
		}
	}
	state(passBy)
	{
		transition
		{

			if( (interSectionPoint == 0 && state_time > 2000.f) || state_time > 3000.f || ((ball.positionField.x() - opponentNearestOnField.x() > 20.f) && (ball.positionRobot.x() - opponentNearestOnRobot.x() > 20.f)))
			{
				if((ball.positionField.x() - opponentNearestOnField.x()) > 20.f)
				{
					WalkToTarget(Pose2f(0.1f, 0.f, 0.6f), Pose2f(ball.positionRobot.angle(), 0.f, ball.positionRobot.y()));
				}
				shootOrNot = striker.getStrikerKickAngle(shootType, ball.positionField,target);
				goto start;
			}
			// if(((ball.positionField.x() - opponentNearestOnField.x()) > 20.f || ball.positionField.x() < robot.x || ball.positionRobot.norm() > 500.f) && (ball.isTeamPositionValid || ball.wasSeen()))
			// {
			// 	shootOrNot = striker.getStrikerKickAngle(shootType, ball.positionField,target);
			// 	goto start;
			// }


		}
		action
		{

			lastStateIsPassBy = true;
			if(ball.positionRobot.norm() > 300.f)
				WalkToBall();
			// else if(ball.positionRobot.norm() > 200.f && obstacle.center.y() < 0.f)
			// 	WalkToTarget(Pose2f(0.3f, 0.5f, 0.5f), Pose2f((theRobotPose.inverse() * target).angle(), ball.positionRobot.x() - 100.f, ball.positionRobot.y() - 100.f));
			// else if(ball.positionRobot.norm() > 200.f && obstacle.center.y() > 0.f)
			// 	WalkToTarget(Pose2f(0.3f, 0.5f, 0.5f), Pose2f((theRobotPose.inverse() * target).angle(), ball.positionRobot.x() - 100.f, ball.positionRobot.y() + 100.f));
			// else if(fabs(ball.positionField.y()) < 2500.f)
			// 	WalkToTarget(Pose2f(0.3f, 0.6f, 0.5f), Pose2f((theRobotPose.inverse() * target).angle(), ball.positionRobot.x() + 100.f, ball.positionRobot.y()));
			// else if(ball.positionRobot.norm() > 200.f && passByLeft == false)//RightPass Ball on Left Foot
		 //        WalkToTarget(Pose2f(.1f, .5f, 0.5f),
		 //                     Pose2f(0.f, ball.positionRobot.x() - 180.f, ball.positionRobot.y()));//考虑能否使用pid进行对球
		 //    else if(ball.positionRobot.norm() > 200.f && passByLeft == true)
		 //        WalkToTarget(Pose2f(.1f, .5f, 0.5f),
		 //                     Pose2f(0.f, ball.positionRobot.x() - 180.f, ball.positionRobot.y() ));
			else if(ball.positionRobot.x() > 200.f && passByLeft == true)
				WalkToTarget(Pose2f(0, 0.4f, 0.6f), Pose2f((theRobotPose.inverse() * PassByTarget).angle(), ball.positionRobot.x() - 150.f, ball.positionRobot.y() + 30.f));
			else if(ball.positionRobot.x() > 200.f && passByLeft == false)
				WalkToTarget(Pose2f(0, 0.4f, 0.6f), Pose2f((theRobotPose.inverse() * PassByTarget).angle(), ball.positionRobot.x() - 150.f, ball.positionRobot.y() - 30.f));
			else if(passByLeft == true && fabs(robot.rotation - (theRobotPose.inverse() * PassByTarget).angle()) < 8_deg)
				WalkToTarget(Pose2f(0.7f, 0.7f, 0.5f), Pose2f((theRobotPose.inverse() * PassByTarget).angle(), ball.positionRobot.x() + 100.f, ball.positionRobot.y() - 100.f));
			else if(passByLeft == true && fabs(robot.rotation - (theRobotPose.inverse() * PassByTarget).angle()) > 8_deg)
				WalkToTarget(Pose2f(0.7f, 0.5f, 0.5f), Pose2f((theRobotPose.inverse() * PassByTarget).angle(), ball.positionRobot.x() + 100.f, ball.positionRobot.y() - 100.f));
			else if(passByLeft == false && fabs(robot.rotation - (theRobotPose.inverse() * PassByTarget).angle()) < 8_deg)
				WalkToTarget(Pose2f(0.7f, 0.7f, 0.5f), Pose2f((theRobotPose.inverse() * PassByTarget).angle(), ball.positionRobot.x() + 100.f, ball.positionRobot.y() + 100.f));
			else if(passByLeft == false && fabs(robot.rotation - (theRobotPose.inverse() * PassByTarget).angle()) > 8_deg)
				WalkToTarget(Pose2f(0.7f, 0.5f, 0.5f), Pose2f((theRobotPose.inverse() * PassByTarget).angle(), ball.positionRobot.x() + 100.f, ball.positionRobot.y() + 100.f));
//			std::cout<<"PassBytarget.x() "<<PassByTarget.x()<<" "<<"PassBytarget.y() "<<PassByTarget.y()<<std::endl;
			// BizhangDribble(target.x(), target.y());
			shootOrNot = striker.getStrikerKickAngle(shootType, ball.positionField,target);
		}
	}
	state(beforeSideKick)
	{
		transition
		{
			if(common.between(ball.positionRobot.x(), sideKickKickX-15.f,sideKickKickX+15.f) && common.between(ball.positionRobot.y(), sideKickKickY-15.f,sideKickKickY+15.f))
				goto sideKick;
		}
		action
		{
			if(ball.positionRobot.norm() > 300.f)
				WalkToBall();
			else if(ball.positionRobot.x() > 200.f)
				WalkToTarget(Pose2f(0, 0.4f, 0.6f), Pose2f(0.f, ball.positionRobot.x() - 150.f, ball.positionRobot.y() - 30.f));

		}
	}
	state(sideKick)
	{
		transition
		{

		}
		action
		{
			Stand();
		}
	}
	state(searchBall)
	{
		transition
		{
			if(ball.isTeamPositionValid || ball.wasSeen())
				goto start;
		}
		action
		{
			if(robot.x > 3200.f)
				SearchForBall();
			else	
				SearchForBallFast();
		}
	}

//  state(goToBallAndKick)
//  {
//    transition
//    {
//      if(action_done)
//        goto start;
//    }
//    action
//    {
//      if(ball.global.x() > 4000)
//        Dribble(5000.f, 0.f);
//      else
//        GoToBallAndKick();
//    }
//  }

	state(kickOffBehavior)
	{
		if(state_time > 10000) //state.time是什么?
		common.isKickOff = false;
		transition
		{
			if(action_done || !common.isKickOff)
			{
				isKickOffLimit = false;
				goto start;
			}
		}
		action
		{
			StrikerKickOff();
		}
	}

}
