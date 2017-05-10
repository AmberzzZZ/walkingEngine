//option(SupporterKickOff)
//{
//  int time = state_time;
//  const float kickX = 180.f;
//  const float kickY = 45.f;
//  bool useRightFoot = ball.positionRobot.y() >= 0? false:true;
//
//  float kickOffAngle = 0.f;
//  std::vector<float> obstacle_x;
//  std::vector<float> obstacle_y;
//
//  obstacle_x.clear();
//  obstacle_y.clear();
//
//  std::cout<<"test------: "<<robot.numStriker<<std::endl;
//  Pose2f StrikerPose = teammate.robotPose(robot.numStriker);
//  std::cout<<"test-----x: "<<StrikerPose.translation.x()<<std::endl;
//  std::cout<<"test-----r: "<<StrikerPose.rotation<<std::endl;
//  for (const Obstacle & obstacle : theObstacleModel.obstacles)
//  {
//	  //obstacle.type == Obstacle::teammate obstacle.type == Obstacle::opponent
//
//	  if( obstacle.type != Obstacle::teammate)
//	  {
////		  obstacle_x.push_back(obstacle.center.x());
////		  obstacle_y.push_back(obstacle.center.y());
//		  obstacle_x.push_back(Geometry::relative2FieldCoord(theRobotPose, obstacle.center).x());
//		  obstacle_y.push_back(Geometry::relative2FieldCoord(theRobotPose, obstacle.center).y());
//	  }
//	  else
//	  {
//		  Vector2f centerOnField = Geometry::relative2FieldCoord(theRobotPose, obstacle.center);
//		  std::cout<<"teammates--x:"<<centerOnField.x()<< "  " <<"teammates--y:"<<centerOnField.y()<<std::endl;
//	  }
//
//  }
//
//  std::vector<float>::iterator biggest = std::max_element(std::begin(obstacle_x),std::end(obstacle_x));
//  std::cout<<"Max element is"<< *biggest << "at position" << std::distance(std::begin(obstacle_x),biggest) << std::endl;
//
//  for (int i = 0;i<obstacle_x.size();i++)
//  {
//	  std::cout<<"obstacles x::"<<obstacle_x[i]<<"   "<<"obstacles y::"<<obstacle_y[i]<<std::endl;
//  }
//
////  behavior.supporter.kickOffDirection = SupporterBehavior::RIGHT;
//
////  if(behavior.supporter.kickOffDirection == SupporterBehavior::LEFT)
////    kickOffAngle = -25_deg;
////  else if(behavior.supporter.kickOffDirection == SupporterBehavior::RIGHT)
////    kickOffAngle = 25_deg;
////
////  std::cout<<"kickOff--> "<<behavior.supporter.kickOffDirection<<std::endl;
//
//  kickOffAngle = 45_deg;
//
//
//  common_transition
//  {
//
//  }
//
//  initial_state(start)
//  {
//    transition
//    {
//      if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
//        goto readyToKickForward;
//      else
//        goto waitForTime;
//    }
//  }
//
//  state(readyToKickForward)
//  {
//    transition
//    {
//      if(time > 3000)
//      {
//        if(!useRightFoot && common.between(ball.positionRobot.y(), kickY -15.f, kickY + 15.f)
//          && common.between(ball.positionRobot.x(), kickX -15.f, kickX+15.f)
//          && fabs(robot.angleToGoal - kickOffAngle) < 8_deg)
//          goto kickForward; //using left foot
//        else if(useRightFoot && common.between(ball.positionRobot.y(), -kickY -15.f, -kickY +15.f)
//          && common.between(ball.positionRobot.x(), kickX -15.f, kickX+15.f)
//          && fabs(robot.angleToGoal - kickOffAngle) < 8_deg)
//          goto kickForward; //using right foot
//      }
//    }
//    action
//    {
//      theHeadControlMode = HeadControl::lookAtBall;
//      if((!useRightFoot && common.between(ball.positionRobot.y(), kickY -10.f, kickY + 10.f)
//         && common.between(ball.positionRobot.x(), kickX -10.f, kickX+5.f)
//         && fabs(robot.angleToGoal - kickOffAngle) < 5_deg)
//         ||
//         (useRightFoot && common.between(ball.positionRobot.y(), -kickY -10.f, -kickY +10.f)
//          && common.between(ball.positionRobot.x(), kickX -10.f, kickX+5.f)
//          && fabs(robot.angleToGoal - kickOffAngle) < 5_deg))
//      {
//        Stand();
//      }
//      else
//      {
//        if(ball.positionRobot.y() >= 0)
//          WalkToTarget(Pose2f(.3f, .5f, .5f),
//                       Pose2f(robot.angleToGoal - kickOffAngle, ball.positionRobot.x() - kickX, ball.positionRobot.y() - kickY));
//        else
//          WalkToTarget(Pose2f(.3f, .5f, .5f),
//                       Pose2f(robot.angleToGoal - kickOffAngle, ball.positionRobot.x() - kickX, ball.positionRobot.y() + kickY));
//      }
//    }
//  }
//
//  state(kickForward)
//  {
//    Vector2f position(2000.f, 0.f);
//    transition
//    {
//      if(action_done || state_time > 3000)
//        goto finish;
//    }
//    action
//    {
////      KickBallToPose(position);
//      if(useRightFoot)
////        InWalkKick(WalkRequest::left, Pose2f(0.f, 0.f, 0.f));
//    	  Kick(useRightFoot, 0.75f);
//      else
////        InWalkKick(WalkRequest::right, Pose2f(0.f, 0.f, 0.f));
//    	  Kick(useRightFoot, 0.75f);
//    }
//  }
//
//  state(waitForTime)
//  {
//    transition
//    {
//      if(state_time > 10000 || ball.speedField.norm() > 50 || ball.positionField.norm() > 300)
//        goto finish;
//    }
//    action
//    {
//      if(common.between(robot.rotation, -30_deg, 30_deg) && common.between(robot.x, -1500, -500) && common.between(robot.y, -500, 500))
//        WalkToPose(robot.strikerReadyPose);
//      else
//        Stand();
//      theHeadControlMode = HeadControl::lookAtBall;
//    }
//  }
//
//  target_state (finish)
//  {
//    action
//    {
//      PlaySound("Nao.wav");
//      theHeadControlMode = HeadControl::lookLeftAndRight;
//      WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.f));
//    }
//  }
//
//}

option(SupporterKickOff)
{
	int time = state_time;
	static float kickX = 180.f;
	static float kickY = 40.f;
	static float kickOffAngle = 0.f;
	static float kickOffTargetPoint_y = 0.f;
	bool useRightFoot = ball.positionRobot.y() >= 0 ? false : true;
//	bool useRightFoot = false;
	bool dribbleMiddle = false;
	bool dribbleKickOff = false;
	int centerCount = 0;
	int leftCount = 0;
	int rightCount = 0;
	bool isCenterClear = false;
	std::cout<<"time:::::::"<<time<<std::endl;
	for(int i = 0;i<6;i++)
	{
		robot.opponentCount[i] = 0;
		robot.opponentFarCount[i] = 0;
		robot.opponentKickoffAngle[i] = 0.f;
	}
	std::vector<float> opponentAngle;
	opponentAngle.clear();
	robot.oppoIndexArea1.clear();
	robot.oppoIndexArea2.clear();
	robot.oppoIndexArea3.clear();
	robot.oppoIndexArea4.clear();
	robot.oppoIndexArea5.clear();
	robot.oppoIndexArea6.clear();

	//Find35 the Obstacle position
	int oppoIndex = 0;
	for (const Obstacle & obstacle : theObstacleModel.obstacles)
	{
//		std::cout<<"---------------------------"<<std::endl;
//		std::cout<<"obstacle.type-->: "<<obstacle.type<<std::endl;
//		std::cout<<"---------------------------"<<std::endl;

		if( obstacle.type != Obstacle::teammate ) //obstacle.type == Obstacle::opponent ||
		{
			Vector2f opponentOnField = Geometry::relative2FieldCoord(theRobotPose, obstacle.center);
			float opponentDistance = sqrt(opponentOnField.x()*opponentOnField.x()+opponentOnField.y()*opponentOnField.y());
			float opponentAngleTemp = atan(opponentOnField.y()/opponentOnField.x());

			//std::cout<<"opponent--x:"<<opponentOnField.x()<< "  " <<"opponent--y:"<<opponentOnField.y()<<std::endl;
			//std::cout<<"opponentDistance--> "<<opponentDistance<<std::endl;
			//std::cout<<"opponent--angle:"<<opponentAngleTemp * 180.f / 3.1415926f<<std::endl;

			if((fabs(opponentAngleTemp) < 10_deg || fabs(opponentOnField.y()) < 200.f) && opponentOnField.x() < 2500.f)
			centerCount++;

			if(opponentOnField.x() < 2000.f && opponentOnField.x() > 0.f)
			{
				opponentAngle.push_back(opponentAngleTemp);
				if(opponentAngleTemp > 40_deg && opponentAngleTemp < 60_deg )
				{
					robot.opponentCount[0] += 2;
					robot.opponentKickoffAngle[0] = opponentAngleTemp;
					robot.oppoIndexArea1.push_back(oppoIndex);
				}
				else if(opponentAngleTemp > 20_deg && opponentAngleTemp < 40_deg )
				{
					robot.opponentCount[1] += 2;
					robot.opponentKickoffAngle[1] = opponentAngleTemp;
					robot.oppoIndexArea2.push_back(oppoIndex);
				}
				else if(opponentAngleTemp > 5_deg && opponentAngleTemp < 20_deg )
				{
					robot.opponentCount[2] += 2;
					robot.opponentKickoffAngle[2] = opponentAngleTemp;
					robot.oppoIndexArea3.push_back(oppoIndex);
				}
				else if(opponentAngleTemp < -5_deg && opponentAngleTemp > -20_deg )
				{
					robot.opponentCount[3] += 2;
					robot.opponentKickoffAngle[3] = opponentAngleTemp;
					robot.oppoIndexArea4.push_back(oppoIndex);
				}
				else if(opponentAngleTemp < -20_deg && opponentAngleTemp > -40_deg )
				{
					robot.opponentCount[4] += 2;
					robot.opponentKickoffAngle[4] = opponentAngleTemp;
					robot.oppoIndexArea5.push_back(oppoIndex);
				}
				else if(opponentAngleTemp < -40_deg && opponentAngleTemp > -60_deg )
				{
					robot.opponentCount[5] += 2;
					robot.opponentKickoffAngle[5] = opponentAngleTemp;
					robot.oppoIndexArea6.push_back(oppoIndex);
				}

			}
//			if(opponentOnField.x() < 500.f && opponentOnField.x() > 0.f && opponentDistance < 750.f)
//			{
////				float oppoAngle = atan(opponentOnField.y()/opponentOnField.x());
////				robot.kickOffAngle = oppoAngle>0?-StrikerAngle+20_deg:-StrikerAngle-20_deg;
//				//std::cout<<"KickAvoidOppo:--> "<<robot.kickOffAngle * 180.f / 3.1415926f<<std::endl;
//			}
			else if(opponentOnField.x() > 2000.f && opponentOnField.x() < 4000.f)
			{
				opponentAngle.push_back(opponentAngleTemp);
				if(opponentAngleTemp > 40_deg && opponentAngleTemp < 60_deg )
				{
					robot.opponentFarCount[0]++;
					robot.oppoIndexArea1.push_back(oppoIndex);
				}
				else if(opponentAngleTemp > 20_deg && opponentAngleTemp < 40_deg )
				{
					robot.opponentFarCount[1]++;
					robot.oppoIndexArea2.push_back(oppoIndex);
				}
				else if(opponentAngleTemp > 5_deg && opponentAngleTemp < 20_deg )
				{
					robot.opponentFarCount[2]++;
					robot.oppoIndexArea3.push_back(oppoIndex);
				}
				else if(opponentAngleTemp < -5_deg && opponentAngleTemp > -20_deg )
				{
					robot.opponentFarCount[3]++;
					robot.oppoIndexArea4.push_back(oppoIndex);
				}
				else if(opponentAngleTemp < -20_deg && opponentAngleTemp > -40_deg )
				{
					robot.opponentFarCount[4]++;
					robot.oppoIndexArea5.push_back(oppoIndex);
				}
				else if(opponentAngleTemp < -40_deg && opponentAngleTemp > -60_deg )
				{
					robot.opponentFarCount[5]++;
					robot.oppoIndexArea6.push_back(oppoIndex);
				}

				//std::cout<<"oppoAngle-->>>> "<<opponentAngleTemp * 180.f / 3.1415926f<<std::endl;
			}
		}
		else if( obstacle.type == Obstacle::teammate )
		{
			Vector2f centerOnField = Geometry::relative2FieldCoord(theRobotPose, obstacle.center);
			std::cout<<"otherobstacles--x:"<<centerOnField.x()<< "  " <<"otherobstacles--y:"<<centerOnField.y()<<std::endl;
		}
		oppoIndex++;
	}

	std::cout<<"---------------------------"<<std::endl;
	std::cout<<"centerCount-->: "<<centerCount<<std::endl;
	std::cout<<"---------------------------"<<std::endl;

	if( centerCount == 0)
	isCenterClear = true;

	for(int j = 0;j<6;j++)
	std::cout<<"robot.opponentCount: "<<robot.opponentCount[j]<<std::endl;
	for(int j = 0;j<6;j++)
	std::cout<<"robot.opponentCountFar: "<<robot.opponentFarCount[j]<<std::endl;

	leftCount = robot.sumOpponentNearLeft() + robot.sumOpponentFarLeft();
	rightCount = robot.sumOpponentNearRight() + robot.sumOpponentFarRight();

//	std::cout<<"---------------------------"<<std::endl;
//	std::cout<<"opponentLeftSum-->: "<<leftCount<<std::endl;
//	std::cout<<"---------------------------"<<std::endl;
//
//	std::cout<<"---------------------------"<<std::endl;
//	std::cout<<"opponentRightSum-->: "<<rightCount<<std::endl;
//	std::cout<<"---------------------------"<<std::endl;

	if( leftCount < rightCount )
	robot.kickOffDirection = -1;//left
	else if(leftCount > rightCount)
	robot.kickOffDirection = 1;//right
	else
	robot.kickOffDirection = 2;

	if( robot.kickOffDirection == -1 )
	{

		if(robot.kickOffType == LibRobot::STABBERIN || robot.kickOffType == LibRobot::BOTHOUT)
		{
			//LightKickorDribble
			robot.kickForce = 0.2f;
			robot.dribbleKickOff = false;
		}
		else
		{
			robot.dribbleKickOff = false;
			robot.kickForce = 0.75f;
		}
		if(robot.opponentCount[2] > 0 && robot.opponentCount[1] == 0)
		{
			robot.kickOffAngle = -40_deg;
			std::cout<<"-40_deg"<<std::endl;
		}
		else if(robot.opponentCount[2] == 0 && robot.opponentCount[1] > 0)
		{
//			robot.kickOffAngle = -20_deg;
			if(fabs(robot.opponentKickoffAngle[1]) > 30_deg )
				robot.kickOffAngle = -robot.opponentKickoffAngle[1]/2;
			else
				robot.kickOffAngle = -(robot.opponentKickoffAngle[1]+40_deg)/2;
			std::cout<<"-20_deg"<<std::endl;
		}
		else if(robot.opponentCount[2] > 0 && robot.opponentCount[1] > 0)
		{
			robot.kickOffAngle = -55_deg;
			//robot.kickForce = 1.0f;
			std::cout<<"-55_deg"<<std::endl;
		}
		else if(robot.opponentCount[0] != 0 )
		{
			robot.kickOffAngle = -robot.opponentKickoffAngle[0]/2;
			std::cout<<"-30_deg"<<std::endl;
		}
		else
		{
			robot.kickOffAngle = -30_deg;
			std::cout<<"-30_deg"<<std::endl;
		}
	}
	else if(robot.kickOffDirection == 1)
	{

		if(robot.kickOffType == LibRobot::STRIKERIN || robot.kickOffType == LibRobot::BOTHOUT )
		{
			//LightKickorDribble
			robot.kickForce = 0.2f;
			robot.dribbleKickOff = false;
		}
		else
		{
			robot.dribbleKickOff = false;
			robot.kickForce = 0.75f;
		}
		if(robot.opponentCount[3] > 0 && robot.opponentCount[4] == 0)
		{
			robot.kickOffAngle = 40_deg;
			std::cout<<"40_deg"<<std::endl;
		}
		else if(robot.opponentCount[3] == 0 && robot.opponentCount[4] > 0)
		{
//			robot.kickOffAngle = 20_deg;
			if(fabs(robot.opponentKickoffAngle[4]) > 30_deg)
				robot.kickOffAngle = -robot.opponentKickoffAngle[4]/2;
			else
				robot.kickOffAngle = -(robot.opponentKickoffAngle[4]+(-40_deg))/2;
			std::cout<<"20_deg"<<std::endl;
		}
		else if(robot.opponentCount[3] > 0 && robot.opponentCount[4] > 0)
		{
			robot.kickOffAngle = 55_deg;
			robot.kickForce = 1.0f;
			std::cout<<"50_deg"<<std::endl;
		}
		else if(robot.opponentCount[5] != 0 )
		{
			robot.kickOffAngle = -robot.opponentKickoffAngle[5]/2;
			std::cout<<"-30_deg"<<std::endl;
		}
		else
		{
			robot.kickOffAngle = 30_deg;
			std::cout<<"50_deg"<<std::endl;
		}
	}
	else if(robot.kickOffDirection == 2)
	{
		if(robot.opponentCount[2] > 0 && robot.opponentCount[1] == 0)
		{
			robot.kickOffAngle = -40_deg;
			std::cout<<"-40_deg"<<std::endl;
		}
		else if(robot.opponentCount[2] == 0 && robot.opponentCount[1] > 0)
		{
//			robot.kickOffAngle = -20_deg;
			robot.kickOffAngle = -robot.opponentKickoffAngle[1]/2;
			std::cout<<"-20_deg"<<std::endl;
		}
		else if(robot.opponentCount[2] > 0 && robot.opponentCount[1] > 0)
		{
			robot.kickOffAngle = -55_deg;
			std::cout<<"-55_deg"<<std::endl;
		}
		else if((robot.opponentCount[0] + robot.opponentCount[1] + robot.opponentCount[2] + robot.opponentCount[3] + robot.opponentCount[4] + robot.opponentCount[5]) == 0)
		robot.kickOffAngle = -25_deg;
		else
		robot.kickOffAngle = -25_deg;
		std::cout<<"Equal"<<std::endl;
		if(robot.kickOffType == LibRobot::STRIKERIN)
		{
			robot.dribbleKickOff = false;
			robot.kickForce = 0.75f;
		}
		else if(robot.kickOffType == LibRobot::STABBERIN)
		{
			robot.dribbleKickOff = false;
			robot.kickForce = 0.75f;
			robot.kickOffAngle = -robot.kickOffAngle;
		}
		else if(robot.kickOffType == LibRobot::BOTHOUT)
		{
			//LightKickOrDribble
			robot.kickForce = 0.2f;
			robot.dribbleKickOff = false;
		}
		else
		{
			robot.dribbleKickOff = false;
			robot.kickForce = 0.75f;
		}
	}
	robot.kickOffTargetPoint.y() = -800.f * tan(robot.kickOffAngle);

//	std::cout<<"---------------------------"<<std::endl;
//	std::cout<<"robot.kickOffDirection-->: "<<robot.kickOffDirection<<std::endl;
//	std::cout<<"---------------------------"<<std::endl;
//
//	std::cout<<"---------------------------"<<std::endl;
//	std::cout<<"robot.kickForce-->: "<<robot.kickForce<<"KickX:----->:"<<kickX<<std::endl;
//	std::cout<<"---------------------------"<<std::endl;
//
//	std::cout<<"---------------------------"<<std::endl;
//	std::cout<<"robot.kickOffTargetPoint.y()-->: "<<robot.kickOffTargetPoint.y()<<std::endl;
//	std::cout<<"---------------------------"<<std::endl;
//
	std::cout<<"---------------------------"<<std::endl;
	std::cout<<"robot.kickOffAngle-->: "<<robot.kickOffAngle * 180.f / 3.1415926f<<std::endl;
	std::cout<<"---------------------------"<<std::endl;
//
//	std::cout<<"---------------------------"<<std::endl;
//	std::cout<<"robot.angleToGoal-->: "<<robot.angleToGoal * 180.f / 3.1415926f<<std::endl;
//	std::cout<<"---------------------------"<<std::endl;
//
//	std::cout<<"---------------------------"<<std::endl;
//	std::cout<<"robot.rotation-->: "<<robot.rotation * 180.f / 3.1415926f<<std::endl;
//	std::cout<<"---------------------------"<<std::endl;
//
//	std::cout<<"---------------------------"<<std::endl;
//	std::cout<<"ball.positionField.x()-->: "<<ball.positionField.x()<<std::endl;
//	std::cout<<"---------------------------"<<std::endl;
//
//	std::cout<<"---------------------------"<<std::endl;
//	std::cout<<"robot.dribbleKickOff-->: "<<robot.dribbleKickOff<<std::endl;
//	std::cout<<"---------------------------"<<std::endl;

	common_transition
	{
		if( ball.positionRobot.norm() > 230.f)
		{
			if(isCenterClear)
			{
				std::cout<<"NoOpponentNear-->>DribbleKickOff!"<<std::endl;
				dribbleMiddle = true;
				robot.kickOffAngle = 0.f;
			}
			else if(robot.dribbleKickOff)
			{
				std::cout<<"SideDribbleKickOff!"<<std::endl;
				dribbleKickOff = true;
			}
			kickOffAngle = robot.kickOffAngle;
			kickOffTargetPoint_y = robot.kickOffTargetPoint.y();
		}

		std::cout<<"kickFoot------------------------------------------------->"<<useRightFoot<<std::endl;

//		std::cout<<"---------------------------"<<std::endl;
//		std::cout<<"dribbleKickOff-->: "<<dribbleKickOff<<std::endl;
//		std::cout<<"---------------------------"<<std::endl;
//
//		std::cout<<"---------------------------"<<std::endl;
//		std::cout<<"dribbleMiddle-->: "<<dribbleMiddle<<std::endl;
//		std::cout<<"---------------------------"<<std::endl;
//
//		std::cout<<"---------------------------"<<std::endl;
//		std::cout<<"robot.kickOffTargetPoint.y()-->: "<<robot.kickOffTargetPoint.y()<<std::endl;
//		std::cout<<"---------------------------"<<std::endl;
	}

	initial_state(start)
	{

		transition
		{

			if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
			{
				if(ball.wasSeen() == true)
				goto walkToBall;
				else
				goto walkToPoint;
			}
			else
			goto waitForTime;
		}
	}

	state(walkToPoint)
	{
		transition
		{
			if(ball.positionRobot.norm() < 250.f || action_done)
			{
				if(dribbleMiddle)
				{
					PlaySound("middledribble.wav");
//					goto dribbleMiddle;
					goto readyToKickForward;
				}
				else if(dribbleKickOff)
				{
					PlaySound("dribble.wav");
					if(robot.kickOffTargetPoint.y() < 0)
					PlaySound("right.wav");
					else
					PlaySound("left.wav");
					goto dribbleKickOff;
				}
				else
				{
					if(kickOffAngle < 0.f)
					PlaySound("leftCHN.wav");
					else
					PlaySound("rightCHN.wav");
					goto readyToKickForward;
				}
			}
		}
		action
		{
			if(ball.wasSeen()==false)
			{
				ball.positionRobot.y() = -robot.y;
				ball.positionRobot.x() = -robot.x;
				std::cout<<"!!!!!!ballwasnotseen!!!!!!"<<std::endl;
			}
			Pose2f point(0_deg, 0.f, 0.f);
			WalkToPoint(point);
		}
	}

	state(walkToBall)
	{
		transition
		{
			if(ball.positionRobot.norm() < 250.f || action_done)
			{
				if(dribbleMiddle)
				{
					PlaySound("middledribble.wav");
//					goto dribbleMiddle;
					goto readyToKickForward;
				}
				if(dribbleKickOff)
				{
					PlaySound("dribble.wav");
					if(robot.kickOffTargetPoint.y() < 0)
					PlaySound("right.wav");
					else
					PlaySound("left.wav");
					goto dribbleKickOff;
				}
				else
				{
					if(kickOffAngle < 0.f)
					PlaySound("leftCHN.wav");
					else
					PlaySound("rightCHN.wav");
					goto readyToKickForward;
				}

			}
		}
		action
		{
			//Pose2f point(0_deg, -240.f, 0.f);
			//WalkToPoint(point);
			if(ball.wasSeen() || ball.isTeamPositionValid)
				theHeadControlMode = HeadControl::lookAtBall;
			else
				theHeadControlMode = HeadControl::lookLeftAndRight;
			WalkToBall(150.f,0.f);
		}
	}
	state(dribbleKickOff)
	{
		transition
		{
			if( ball.positionField.x() > 750 )
			goto finish;
		}
		action
		{
			Dribble(robot.kickOffTargetPoint.x(),kickOffTargetPoint_y);
		}
	}

	state(dribbleMiddle)
	{
		transition
		{
			if( ball.positionField.x() > 750 || action_done)
			goto finish;
		}
		action
		{
			Dribble(800.f,0.f);
		}
	}

	state(readyToKickForward)
	{
		transition
		{

			if(true)
			{
				if(!useRightFoot && common.between(ball.positionRobot.y(), kickY -20.f, kickY + 20.f)
						&& common.between(ball.positionRobot.x(), kickX -20.f, kickX+20.f)
						&& fabs(robot.angleToGoal - kickOffAngle) < 6_deg)
				goto kickForward; //using left foot
				else if(useRightFoot && common.between(ball.positionRobot.y(), -kickY -20.f, -kickY +20.f)
						&& common.between(ball.positionRobot.x(), kickX -20.f, kickX+20.f)
						&& fabs(robot.angleToGoal - kickOffAngle) < 6_deg)
				goto kickForward;//using right foot
			}
		}
		action
		{


			theHeadControlMode = HeadControl::lookAtBall;
			if((!useRightFoot && common.between(ball.positionRobot.y(), kickY -15.f, kickY + 15.f)
							&& common.between(ball.positionRobot.x(), kickX -15.f, kickX + 15.f)
							&& fabs(robot.angleToGoal - kickOffAngle) < 5_deg)
					||
					(useRightFoot && common.between(ball.positionRobot.y(), -kickY -15.f, -kickY +15.f)
							&& common.between(ball.positionRobot.x(), kickX -15.f, kickX + 15.f)
							&& fabs(robot.angleToGoal - kickOffAngle) < 5_deg))
			{
//				Stand();
			}
			else
			{
//				if(ball.positionRobot.y() >= 0)
				WalkToTarget(Pose2f(.5f, .7f, .4f),
						Pose2f(robot.angleToGoal - kickOffAngle, ball.positionRobot.x() - kickX, ball.positionRobot.y() - kickY));
//				else
//				WalkToTarget(Pose2f(.5f, .7f, .4f),
//						Pose2f(robot.angleToGoal - kickOffAngle, ball.positionRobot.x() - kickX, ball.positionRobot.y() + kickY));
			}
		}
	}



	state(kickForward)
	{
		Vector2f position(2000.f, 0.f);
		transition
		{
			if(action_done || state_time > 3000)
			goto finish;
		}
		action
		{
//      KickBallToPose(position);
			if(useRightFoot)
//        InWalkKick(WalkRequest::left, Pose2f(0.f, 0.f, 0.f));
			{

				std::cout<<"robot.angleToGoal:--> "<<robot.angleToGoal* 180.f / 3.1415926f<<std::endl;
//				Kick(useRightFoot);
				FlashKick(useRightFoot);
			}
			else
//        InWalkKick(WalkRequest::right, Pose2f(0.f, 0.f, 0.f));
			{

				std::cout<<"robot.angleToGoal:--> "<<robot.angleToGoal* 180.f / 3.1415926f<<std::endl;
//				Kick(useRightFoot);
				FlashKick(useRightFoot);
			}
		}
	}

	state(waitForTime)
	{
		transition
		{
			if(state_time > 10000 || ball.speedField.norm() > 50 || ball.positionField.norm() > 300)
			goto finish;
		}
		action
		{
			if(common.between(robot.rotation, -30_deg, 30_deg) && common.between(robot.x, -1500, -500) && common.between(robot.y, -500, 500))
			WalkToPose(robot.strikerReadyPose);
			else
			Stand();
			theHeadControlMode = HeadControl::lookAtBall;
		}
	}

	target_state (finish)
	{
		action
		{
			PlaySound("Nao.wav");
			theHeadControlMode = HeadControl::lookLeftAndRight;
			WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.f));

		}
	}

}

