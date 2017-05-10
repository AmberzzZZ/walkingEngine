/*
 *@Usage:Ball-searching method used by striker,supporter and stabber
 */
option(SearchForBallFast)//是否完整转一圈
{
  bool turnLeft = false;
  static unsigned int rotation_circle = 0;
 Vector2f refPoint = Vector2f(4500.f,0.f);
 Vector2f refPointRelative;
     refPointRelative = theRobotPose.inverse() * refPoint;
 Angle refAngle;
     refAngle = refPointRelative.angle(); //机器人坐标系 球门的角度

  // if(refAngle > 0)
  //   turnLeft = false;
  // else
  //   turnLeft = true;

  initial_state(lookAround)
  {
    transition
    {
//      if(state_time > 500 && ball.isTeamPositionValid)
//        goto lookAtTeamBall;
//      else if(state_time > 500)
        goto search;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRightDown;
    }
  }

  state(lookAtTeamBall)
  {
    transition
    {
      if(action_done && !ball.wasSeen()) //TODO
        goto searchForBallTogether;
    }
    action
    {
      if(std::abs((theRobotPose.inverse()
          * Vector2f(ball.teamPosition.x(), ball.teamPosition.y())).angle())
         > (80.f/180.f*pi))
        LookLeftAndRight(35_deg, 20_deg);
      else
        theHeadControlMode = HeadControl::lookAtTeamBall;
//        theHeadControlMode = HeadControl::lookAtBall;
      if((Geometry::fieldCoord2Relative(
          theRobotPose, Vector2f(ball.teamPosition.x(),
                                 ball.teamPosition.y()))).y() > 0)
      {
    	  TurnOneRound(false);
        turnLeft = false;
      }
      else
      {
    	  TurnOneRound(true);
        turnLeft = true;
      }
    }
  }

  state(search)
  {
    transition
    {
    	if(action_done)
    	{
    		goto searchForBallTogether;
    	}
    }
    action
    {
//      theHeadControlMode = state_time % 6000 < 3000? HeadControl::lookLeftAndRightDown : HeadControl::lookLeftAndRightFast;
//      theHeadControlMode = state_time % 6000 < 3000? HeadControl::lookDown : HeadControl::lookForward;
      if(turnLeft)
      {
//        WalkAtSpeedPercentage(Pose2f(-0.7f, 0.f, 0.f));
    	  TurnThreePart(true);
      }
      else
      {
//        WalkAtSpeedPercentage(Pose2f(0.7f, 0.f, 0.f));
    	  TurnThreePart(false);
      }
    }
  }
  state(searchForBallTogether)
  {
	  transition
	  {

	  }
	  action
	  {
		  SearchForBallTogether();
	  }
  }
}
