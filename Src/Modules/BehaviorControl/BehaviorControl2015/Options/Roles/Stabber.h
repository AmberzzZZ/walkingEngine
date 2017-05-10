option(Stabber)
{
//stabber's best choice now
	static bool isNearLine=false;
	//static var is available.
// 	static float a=0.f;
// 	a++;
// 	std::cout<<"a="<<a<<std::endl;
	double k=0;
	double b=0;
	double d=0;
	int leftRight;
	Vector2f strikerToBall(ball.positionField.x()-teammate.striker.pose.translation.x(),ball.positionField.y()-teammate.striker.pose.translation.y());
	float attackPoseX = 3700.f;
  	float attackPoseY = 1300.f;
  	float robotXWhenIsNearLine=0.f;        //used when near shootline
  	float robotYWhenIsNearLine=0.f;
  	
  	float attackAngle = 0.f;
//  	Vector2f attackPoint(3700.f,1300.f);
//obstacle in field
  	float obstacleXField = 0.f;
  	float obstacleYField=  0.f;

//info of opponent goal
	const float opponentGoalX=4500.f;
	const float opponentGoalY=0.f;
	const Vector2f opponentGoal(opponentGoalX,opponentGoalY);
	Vector2f ballToOppGoal(opponentGoalX-ball.positionField.x(),opponentGoalY-ball.positionField.y());
  	
//six possible position where stabber will stay. If there is an obstacle, stabber will choose a better position.
	const Vector2f pointNo1Left(3000.f,1300.f);
	const Vector2f pointNo2Left(2300.f,1300.f);
	const Vector2f pointNo3Left(2300.f,800.f);
	const Vector2f pointNo1Right(3000.f,-1300.f);
	const Vector2f pointNo2Right(2300.f,-1300.f);
	const Vector2f pointNo3Right(2300.f,-800.f);
//six angles of listed six points to the opponent goal	
	const float pointNo1LeftToGoal=(opponentGoal-pointNo1Left).angle()-40_deg;
	const float pointNo2LeftToGoal=(opponentGoal-pointNo2Left).angle()-40_deg;
	const float pointNo3LeftToGoal=(opponentGoal-pointNo3Left).angle()-40_deg;
	const float pointNo1RightToGoal=(opponentGoal-pointNo1Right).angle()+40_deg;
	const float pointNo2RightToGoal=(opponentGoal-pointNo2Right).angle()+40_deg;
	const float pointNo3RightToGoal=(opponentGoal-pointNo3Right).angle()+40_deg;
// 	std::cout<<"-------------anglesangles-----------"<<std::endl;
// 	std::cout<<pointNo1LeftToGoal<<std::endl;
// 	std::cout<<pointNo2LeftToGoal<<std::endl;
// 	std::cout<<pointNo3LeftToGoal<<std::endl;
// 	std::cout<<pointNo1RightToGoal<<std::endl;
// 	std::cout<<pointNo2RightToGoal<<std::endl;
// 	std::cout<<pointNo3RightToGoal<<std::endl;
// 	std::cout<<"-------------anglesangles-----------"<<std::endl;

//obstacles	
	std::vector<Vector2f> possibleObstacles;
	for (const Obstacle & obstacle : theObstacleModel.obstacles)
	{
	
	 	obstacleXField=obstacle.center.x()*cos(robot.rotation)-obstacle.center.y()*sin(robot.rotation)+robot.x;
 		obstacleYField=obstacle.center.x()*sin(robot.rotation)+obstacle.center.y()*cos(robot.rotation)+robot.y;
		possibleObstacles.push_back(Vector2f(obstacleXField,obstacleYField));
// 		std::cout<<"---------------------------"<<std::endl;
// 		std::cout<<"obstacle.type-->: "<<obstacle.type<<std::endl;
//  		std::cout<<"---------------------------"<<std::endl;
//  		std::cout<<"obstacle.center on field-->:"<<std::endl;
// 	  	std::cout<<obstacleXField<<' '<<obstacleYField<<std::endl;
//  	std::cout<<"obstacle.center on robot-->:"<<std::endl<<obstacle.center.x()<<std::endl;
//  	std::cout<<obstacle.center.y()<<std::endl;
		
	}
//whether there is an obstacle in six different points
	int isPointOccupiedFlag[6]={0};    //initially, there are no obstacles on chosen points
//choose best point	
	if(robot.y>0)
	{
		isPointOccupiedFlag[0]=stabber.isPointOccupied(pointNo1Left,possibleObstacles);
		isPointOccupiedFlag[1]=stabber.isPointOccupied(pointNo2Left,possibleObstacles);
		isPointOccupiedFlag[2]=stabber.isPointOccupied(pointNo3Left,possibleObstacles);
// 		std::cout<<"---------------------------------"<<std::endl;
// 		std::cout<<"robot position:"<<std::endl;
// 		std::cout<<robot.x<<"   "<<robot.y<<std::endl;
// 		std::cout<<"robot rotation:"<<robot.rotation<<std::endl;
// 		std::cout<<"isPointOccupiedFlag[0]:"<<isPointOccupiedFlag[0]<<std::endl;
// 		std::cout<<"isPointOccupiedFlag[1]:"<<isPointOccupiedFlag[1]<<std::endl;
// 		std::cout<<"isPointOccupiedFlag[2]:"<<isPointOccupiedFlag[2]<<std::endl;
		if(isPointOccupiedFlag[0]==0)
		{
			attackPoseX=pointNo1Left.x();
			attackPoseY=pointNo1Left.y();
			attackAngle=pointNo1LeftToGoal;
		}
		else if(isPointOccupiedFlag[1]==0)
		{
			attackPoseX=pointNo2Left.x();
			attackPoseY=pointNo2Left.y();
			attackAngle=pointNo2LeftToGoal;
		}else
		{
			attackPoseX=pointNo3Left.x();
			attackPoseY=pointNo3Left.y();
			attackAngle=pointNo3LeftToGoal;
		}
	}
	else
	{
		isPointOccupiedFlag[3]=stabber.isPointOccupied(pointNo1Right,possibleObstacles);
		isPointOccupiedFlag[4]=stabber.isPointOccupied(pointNo2Right,possibleObstacles);
		isPointOccupiedFlag[5]=stabber.isPointOccupied(pointNo3Right,possibleObstacles);
		
// 		std::cout<<"---------------------------------"<<std::endl;
// 		std::cout<<"isPointOccupiedFlag[3]:"<<isPointOccupiedFlag[3]<<std::endl;
// 		std::cout<<"isPointOccupiedFlag[4]:"<<isPointOccupiedFlag[4]<<std::endl;
// 		std::cout<<"isPointOccupiedFlag[5]:"<<isPointOccupiedFlag[5]<<std::endl;
		
		if(isPointOccupiedFlag[3]==0)
		{
			attackPoseX=pointNo1Right.x();
			attackPoseY=pointNo1Right.y();
			attackAngle=pointNo1RightToGoal;
		}
		else if(isPointOccupiedFlag[4]==0)
		{
			attackPoseX=pointNo2Right.x();
			attackPoseY=pointNo2Right.y();
			attackAngle=pointNo2RightToGoal;
		}else
		{
			attackPoseX=pointNo3Right.x();
			attackPoseY=pointNo3Right.y();
			attackAngle=pointNo3RightToGoal;
		}
	}  

//  	std::cout<<"-------------attack-1----------"<<std::endl;
//  	std::cout<<attackAngle<<std::endl;
//  	std::cout<<attackPoseX<<std::endl;
//  	std::cout<<attackPoseY<<std::endl;
//  	std::cout<<"-------------attack-1----------"<<std::endl;

	float strikerPoseX = teammate.striker.pose.translation.x();
    float strikerPoseY = teammate.striker.pose.translation.y();
    if( strikerPoseX > 2000.f)
    {
    	if(strikerPoseY * robot.y >0) 
    	{
    		attackPoseY = -attackPoseY;
    	}
    }

	if(behavior.striker.isControlBall == true && strikerToBall.norm()<600.f && robot.x>teammate.striker.pose.translation.x() && (!stabber.isTooCloseToStriker()))
  	{
  		k=(0-ball.positionField.y())/(4500.f-ball.positionField.x());
  		b=-4500.f*k;
  		d=fabs(k*robot.x-robot.y+b)/sqrt(1+k*k);
  		if(d<400.f)
  		{
  			isNearLine=true;
  			robotXWhenIsNearLine=robot.x;
  			robotYWhenIsNearLine=robot.y;
//  			std::cout<<"-------------para when near shoot line----------"<<std::endl;
//  			std::cout<<"robotXWhenIsNearLine"<<std::endl;
// 			std::cout<<robotXWhenIsNearLine<<std::endl;
// 			std::cout<<"robotYWhenIsNearLine"<<std::endl;
//		 	std::cout<<robotYWhenIsNearLine<<std::endl;
//		 	std::cout<<"-------------para when near shoot line----------"<<std::endl;
  		}
  	}
  	
  	
//  	std::cout<<"-------------striker control ball----------"<<std::endl;
//  	std::cout<<"striker:control?"<<behavior.striker.isControlBall<<std::endl;
//  	std::cout<<"striker to ball:"<<strikerToBall.norm()<<std::endl;
//  	std::cout<<"robot.x:"<<robot.x<<std::endl;
//  	std::cout<<"striker.x"<<teammate.striker.pose.translation.x()<<std::endl;
//  	std::cout<<"-------------striker control ball----------"<<std::endl;
//  	std::cout<<"-------------isNearShootLine----------"<<std::endl;
//  	std::cout<<"k="<<k<<"b="<<b<<std::endl;
//  	std::cout<<"d="<<d<<std::endl;
//  	std::cout<<"isNearLine="<<isNearLine<<std::endl;
//  	std::cout<<"-------------isNearShootLine----------"<<std::endl;
  	
  	
	float strikerBallTimeNotSeen = 0;
	for(const Teammate &teammate : theTeammateData.teammates)
  	{
    	if(teammate.role.role != Role::striker)
      		continue;
    	strikerBallTimeNotSeen = theFrameInfo.getTimeSince(teammate.ball.timeWhenLastSeen);
  	}
	common_transition
  	{
// 		std::cout<<"-------------striker or not----------"<<std::endl;
// 		std::cout<<"ball.positionRobot.norm()"<<std::endl;
// 		std::cout<<ball.positionRobot.norm()<<std::endl;
// 		std::cout<<"teammate.striker.ballDistance - 500.f"<<std::endl;
// 		std::cout<<teammate.striker.ballDistance - 500.f<<std::endl;
// 		std::cout<<"strikerBallTimeNotSeen"<<std::endl;
// 		std::cout<<strikerBallTimeNotSeen<<std::endl;
// 		std::cout<<"ball.notSeenTime()"<<std::endl;
// 		std::cout<<ball.notSeenTime()<<std::endl;
// 		std::cout<<"-------------striker or not----------"<<std::endl;
    	if(common.isKickOff)
      		goto kickOff;
    	if((ball.positionRobot.norm() < teammate.striker.ballDistance - 500.f || strikerBallTimeNotSeen > 6000) && ball.notSeenTime() < 1000)
      		{
      			//std::cout<<"stabber became striker!!!!!!!!!!!!!!!!"<<std::endl;
      			behavior.stabberOutput.requestRoleType = Role::striker;
      		}
    	if(stabber.isTooCloseToStriker())
      		goto avoidStriker;
    	if(ball.notSeenTime() > 2000.f && !ball.isTeamPositionValid)
    		goto searchForBall;
  	}
	
	initial_state(start)
	{
		transition
		{
			goto walkToDestination;
		}
		action
		{
			
		}
	}
	state(walkToDestination)
	{
		transition
		{
			Vector2f disRobotToTarget(attackPoseX-robot.x,attackPoseY-robot.y);
// 			std::cout<<"-----------disRobotToTarget.norm() threshold-200----------"<<std::endl;
// 			std::cout<<disRobotToTarget.norm()<<std::endl;
// 			std::cout<<"-----------disRobotToTarget.norm() threshold-200----------"<<std::endl;
// 			std::cout<<"----------- robot.rotation-attackAngle----------"<<std::endl;
// 			std::cout<<robot.rotation-attackAngle<<std::endl;
// 			std::cout<<"-----------robot.rotation-attackAngle----------"<<std::endl;
			if(disRobotToTarget.norm() < 100.f && (fabs(robot.rotation-attackAngle)) < 10_deg)
				goto wait;
// 			else if(isNearLine == true && disRobotToTarget.norm() > 400.f )
			else if(isNearLine == true && robot.x<1500.f && robot.x>0)
 				goto avoidShootLine;
		}
		action
		{
			PlaySound("go.wav");
// 			std::cout<<"-------------attack-2----------"<<std::endl;
//  			std::cout<<attackAngle<<std::endl;
//  			std::cout<<attackPoseX<<std::endl;
//  			std::cout<<attackPoseY<<std::endl;
//  			std::cout<<"-------------attack-2----------"<<std::endl;
			WalkToPose(Pose2f(attackAngle,attackPoseX,attackPoseY));
		}
	}
	state(wait)
  	{
    	transition
   		{
   			
    		Vector2f distance(attackPoseX-robot.x,attackPoseY-robot.y);
    	//	if(distance.norm()>350.f && (fabs(robot.rotation-attackAngle))>20_deg)
    		if(distance.norm()>300.f)
    			goto walkToDestination;
    		else if(isNearLine == true)
			{
				goto standBack;
			}
    		else
    		{
// 	    		std::cout<<"-------------wait-for-dis----------"<<std::endl;
// 	 			std::cout<<distance.norm()<<std::endl;
// 	 			std::cout<<"-------------wait-for-dis----------"<<std::endl;
 			}
    	}
	    action
	    {
	    	if(ball.wasSeen() || ball.isTeamPositionValid)
	    	{
				theHeadControlMode = state_time % 4000 < 2000?HeadControl::lookAtBall:HeadControl::lookLeftAndRight;//判断条件是否改为是否看到了球
			}
			else
			{
				theHeadControlMode = HeadControl::lookLeftAndRight;
			}
			behavior.stabberOutput.isControlBall = false;
		    Stand();
	    }
  	}

  	state(standBack)
  	{
  		transition
  		{
  			if(robot.y<-2700.f || robot.y>2700.f || (fabs(robot.y- attackPoseY)>400.f))
  			{
  				isNearLine=false;
				goto waitForAWhile;
			}
  		}
  		action
  		{
  			PlaySound("standback.wav");
  			WalkAtSpeedPercentage(Pose2f(0.f, -0.5f, 0.f));
  		}
  	}

/*
* @brief    seperated parts named one, two, three, four
* @usage    in avoidShootLine
-------------------
|  tjark keeper   |
|                 |
|                 |
|                 |
-------------------
| ]  two | three[ |
|   ]    |    [   |
|     ]  |  [     |
| one    ] [ four |
-------------------
 */
  	state(avoidShootLine)
  	{
  		Vector2f disRobotToTarget(attackPoseX-robot.x,attackPoseY-robot.y);
  		transition
  		{
  			if(robot.y<-2700.f || robot.y>2700.f || (fabs(robot.y-robotYWhenIsNearLine)>400.f))
  			{
  				isNearLine=false;
				goto walkToDestination;
			}
  		}
  		action
  		{  
  			PlaySound("intheway.wav");	
  			if(k*robot.x-robot.y+b>0)
  			{
  				std::cout<<"move down!!!"<<std::endl;
  				if(k>0)    //part one
  				{
  				// 	std::cout<<"-------------disRobotToTarget----------"<<std::endl;
	 				// std::cout<<disRobotToTarget.norm()<<std::endl;
	 				// std::cout<<"-------------disRobotToTarget----------"<<std::endl;
//   					if((disRobotToTarget.angle()-attackAngle)<20_deg && disRobotToTarget.norm()<300.f)
//   					{
//   						WalkAtSpeedPercentage(Pose2f(0.f, -0.5f, 0.f));
//   					}
//   					else 
  					{
  						WalkToPose(Pose2f(45_deg,robot.x+600.f,robotYWhenIsNearLine-600.f));//这里的角度没用，因为根据距离判断条件会跳到waitforawhile
  					}
  				}else      //part three
  				{
  				// 	std::cout<<"-------------disRobotToTarget----------"<<std::endl;
	 				// std::cout<<disRobotToTarget.norm()<<std::endl;
	 				// std::cout<<"-------------disRobotToTarget----------"<<std::endl;
//   					if((disRobotToTarget.angle()-attackAngle)<20_deg && disRobotToTarget.norm()<300.f)
//   					{
//   						WalkAtSpeedPercentage(Pose2f(0.f, -0.5f, 0.f));
//   					}
//   					else
  					{
  						WalkToPose(Pose2f(-45_deg,robot.x+600.f,robotYWhenIsNearLine-600.f));   //这里的角度没用，因为根据距离判断条件会跳到waitforawhile
  					}
  				}
  			}else
  			{
  			    std::cout<<"move up!!!"<<std::endl;
  			    if(k>0)    //part two
  				{
  				// 	std::cout<<"-------------disRobotToTarget----------"<<std::endl;
	 				// std::cout<<disRobotToTarget.norm()<<std::endl;
	 				// std::cout<<"-------------disRobotToTarget----------"<<std::endl;
//   					if((disRobotToTarget.angle()-attackAngle)<20_deg && disRobotToTarget.norm()<300.f)
//   					{
//   						WalkAtSpeedPercentage(Pose2f(0.f, 0.5f, 0.f));
//   					}
//   					else 
  					{
  						WalkToPose(Pose2f(45_deg,robot.x+600.f,robotYWhenIsNearLine+600.f));//这里的角度没用，因为根据距离判断条件会跳到waitforawhile
  					}
  				}else      //part four
  				{
  				// 	std::cout<<"-------------disRobotToTarget----------"<<std::endl;
	 				// std::cout<<disRobotToTarget.norm()<<std::endl;
	 				// std::cout<<"-------------disRobotToTarget----------"<<std::endl;
//   					if((disRobotToTarget.angle()-attackAngle)<20_deg && disRobotToTarget.norm()<300.f)
//   					{
//   						WalkAtSpeedPercentage(Pose2f(0.f, -0.5f, 0.f));
//   					}
//   					else
  					{
  						WalkToPose(Pose2f(-45_deg,robot.x+600.f,robotYWhenIsNearLine+600.f));//这里的角度没用，因为根据距离判断条件会跳到waitforawhile
  					}
  				}
  			}
  		}
  	}
  	
  	state(waitForAWhile)
  	{
  		transition
  		{
//   		std::cout<<"-------------state_time----------"<<std::endl;
//  	 	std::cout<<state_time<<std::endl;
//  	 	std::cout<<"-------------state_time----------"<<std::endl;
  			if(state_time>2000)
  			{
  				goto walkToDestination;
  			}
  		}
  		action
  		{
  			behavior.stabberOutput.isControlBall = false;
			if(ball.wasSeen() || ball.isTeamPositionValid)
			{
				theHeadControlMode = state_time % 4000 < 2000?HeadControl::lookAtBall:HeadControl::lookLeftAndRight; //判断条件是否改为是否看到了球
			}
			else
			{
				theHeadControlMode = HeadControl::lookLeftAndRight;
			}
  			Stand();
  		}
  	}
  
    state(avoidStriker)
    {
    	isNearLine=false;
    	float angleToStriker = (robot.pose.inverse()* teammate.striker.pose.translation).angle();
    	float distanceToStriker = (teammate.striker.pose.translation - robot.pose.translation).norm();
    	float yToStriker = (robot.pose.inverse()* teammate.striker.pose.translation).y();
    	transition
    	{
      		if(distanceToStriker > 600 && !stabber.isTooCloseToStriker())
        		goto start;
    	}
    	action
    	{
// 	        PlaySound("allright.wav");
// 	        Stand();
	        theHeadControlMode = state_time % 6000 < 3000? HeadControl::lookAtBall : HeadControl::lookLeftAndRight;
	        if(yToStriker > 0)
	            WalkAtSpeedPercentage(Pose2f(0.f, 0.f, -0.7f));//如果striker在左边，则向右走
	        else
	            WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.7f));
	    }
  	}

	state(kickOff)
	{
		isNearLine=false;
	    if(state_time > 10000)
	        common.isKickOff = false;
	    transition
	    {
	      	if(action_done || !common.isKickOff)
	        	goto start;
	    }
	    action
	    {
	      	StabberKickOff();
	    }
	}
	state(searchForBall)
	{
		transition
		{
			if(ball.wasSeen() || ball.isTeamPositionValid)
			{
				goto start;
			}
		}
		action
		{
			SearchForBallFast();
		}
	}
}

