option(StrikerKickOff)
{

  common_transition
  {

  }

  initial_state(start)
  {
    transition
    {
      if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
        goto walkForward;
      else
        goto wait;
    }
  }

  state(walkForward)
  {
	std::cout<<"striker-y: "<<robot.y<<std::endl;
	Pose2f pose_left(-90_deg, 1500.f, 1500.f);
	Pose2f pose_right(90_deg, 1500.f, -1500.f);
    transition
    {
      if(action_done || !common.isKickOff)
        goto finish;
    }
    action
    {
//      if(robot.y > 0)
    	WalkToPose(pose_left);
//      else
//    	WalkToPose(pose_right);
    }
  }

  state(wait)
  {
    transition
    {

    }
    action
    {
      if(ball.positionRobot.norm() < 1500.f)
        TurnToBall();
      else
        WalkToBall();
    }
  }

  target_state(finish)
  {
    transition
    {

    }
    action
    {

    }
  }
}
