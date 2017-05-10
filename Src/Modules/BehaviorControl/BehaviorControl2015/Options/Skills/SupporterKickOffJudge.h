option(SupporterKickOffJudge)
{

	//Find the Striker position

//	float StrikerAngle = 0.f;

//	float StrikerAngle_deg = 0.f;
//	StrikerAngle = atan(StrikerPose.translation.y()/StrikerPose.translation.x());
//	StrikerAngle_deg = StrikerAngle * 180.f / 3.1415926f;
//	std::cout<<"StrikerAngle: "<<StrikerAngle<<"----"<<"StrikerAngle_deg: "<<StrikerAngle_deg<<std::endl;

	std::cout<<std::endl<<"-------------------"<<std::endl;
	Pose2f StrikerPose = teammate.robotPose(robot.numStriker);
	std::cout<<"numStriker: "<<robot.numStriker<<std::endl;
	std::cout<<"Striker-----x: "<<StrikerPose.translation.x()<<' '<<"Striker-----y: "<<StrikerPose.translation.y()<<std::endl;

	//Find the Stabber position
	std::cout<<std::endl<<"-------------------"<<std::endl;
	Pose2f StabberPose = teammate.robotPose(robot.numStabber);
	std::cout<<"numStabber: "<<robot.numStabber<<std::endl;
	std::cout<<"Stabber-----x: "<<StabberPose.translation.x()<<' '<<"Stabber-----y: "<<StabberPose.translation.y()<<std::endl;

	std::cout<<std::endl<<"-------------------"<<std::endl;
	std::cout<<"StabberIsOnlineOrNot  "<<teammate.stabber.isOnline<<std::endl;
	std::cout<<std::endl<<"-------------------"<<std::endl;

	std::cout<<std::endl<<"-------------------"<<std::endl;
	std::cout<<"StrikerIsOnlineOrNot  "<<teammate.striker.isOnline<<std::endl;
	std::cout<<std::endl<<"-------------------"<<std::endl;

	if(!robot.isKickOffTypeConfirm)
	{

		if(StrikerPose.translation.x() > -1500.f && StrikerPose.translation.x() < 0.f && StabberPose.translation.x() > -1500.f && StabberPose.translation.x() < 0.f)
			robot.kickOffType = LibRobot::BOTHIN;
		else if(StrikerPose.translation.x() > -1500.f && StrikerPose.translation.x() < 0.f )
			robot.kickOffType = LibRobot::STRIKERIN;
		else if(StabberPose.translation.x() > -1500.f && StabberPose.translation.x() < 0.f )
			robot.kickOffType = LibRobot::STABBERIN;
		else
			robot.kickOffType = LibRobot::BOTHOUT;

		robot.isKickOffTypeConfirm = true;

		if(robot.kickOffType == LibRobot::STRIKERIN)
		{
			PlaySound("striker.wav");
		}
		else if(robot.kickOffType == LibRobot::STABBERIN)
		{
			PlaySound("stabber.wav");
		}
		else if(robot.kickOffType == LibRobot::BOTHOUT)
		{
			PlaySound("supporter.wav");
		}
		else if(robot.kickOffType = LibRobot::BOTHIN)
		{
			PlaySound("heheCHN.wav");
		}
		else
			PlaySound("tjark.wav");
	}


	std::cout<<"---------------------------"<<std::endl;
	std::cout<<"robot.kickOffType-->: "<<robot.kickOffType<<std::endl;
	std::cout<<"---------------------------"<<std::endl;

	initial_state(start)
	{
		transition
		{
			if(action_done)
			goto finish;
		}
		action
		{
			if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
			{
				SupporterKickOff();
			}
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

