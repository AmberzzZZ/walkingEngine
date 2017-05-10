option(PlayingState)
{
  initial_state(learning)
  {
    transition
    {
//      if(state_time > 2000)
        goto play;
    }
    action
    {
      Stand();
      theHeadControlMode = HeadControl::lookForward;
    }
  }
  state(play)
  {
	  transition
	  {
//		  if(state_time > 8000.f)
//			  goto learning;
	  }
    action
    {
//    	std::cout<<"ball.speed.x()--->"<<ball.speedRobot.x()<<std::endl;
//    	if(ball.notSeenTime() > 2000.f)
//			LookLeftAndRightFast();
//    	else
//    		LookAtBall();
//    	SpecialAction(SpecialActionRequest::goalkeeperDefend,true);
//      Stand();
//      if(theRobotInfo.number == 1)
//    	      Keeper();
//     Striker();
//      Supporter2();
//    	if(fabs(ball.speedRobot.x()) < 20.f)
//    	{
//			std::cout<<"speed--->"<<ball.speedRobot.x()<<std::endl;
//			std::cout<<"endpoint--->"<<ball.endPosition.x()<<std::endl;
//    	}
//    	Stand();
//    	LookLeftAndRight();
//    	if(ball.wasSeen())
//    	{
//    		LookAtBall();
//    	}
//    	else
//    	{
//    		LookLeftAndRight();
//    	}
//		std::cout<<"ball.endPosition.x---->"<<ball.endPosition.x()<<std::endl;
//		std::cout<<"ball.speedRobot.x---->"<<ball.speedRobot.x()<<std::endl;
//      Defender();
//      Stabber();
//      RoleAssign();
//    	Dribble(4500.f, 0.f);
      Tester();
//    	WalkToBall();
//      WalkAtSpeedPercentage(Pose2f(0.f, 1.f, 0.f));
//      LookForward();
//    	WalkToTarget(Pose2f(0.7f, 0.f, 0.f), Pose2f(179_deg, 0.f, 0.f));
    }
  }
}
