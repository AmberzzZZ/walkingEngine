
/** A role for code develop and debug */
option(Tester)
{
  const float inWalkKickForwardX = 175.f;
  const float inWalkKickForwardY = 45.f;

  const float inWalkKickSideX = 150.f;
  const float inWalkKickSideY = -15.f;
  Vector2f vector2Goal = theRobotPose.inverse() * Vector2f(4500.f, 0.f);
  Angle angleToShoot = vector2Goal.angle();
  std::cout<<"angleToShoot------>"<<57.3*(angleToShoot)<<std::endl;
  angleToShoot.normalize();
  Vector2f ballField = theRobotPose * ball.positionRobot;
  initial_state(start)
  {
    transition
    {    
//		if(common.between(theRobotPose.translation.x(), -20.f, 20.f)
//				&& common.between(theRobotPose.translation.y(), -20.f, 20.f)
//				&& common.between(theRobotPose.rotation, -5_deg, 5_deg))
//		{
//			goto stand;
//		}
    	if(ball.positionRobot.norm() < 300.f && fabs(angleToShoot) < 20_deg)
    	{
    		goto dribble;
    	}
    }
    action
    {
      /* skill test */
//      theHeadControlMode = HeadControl::lookLeftAndRight;
//      Stand();
//      SimpleKick(0);
//      GoToReadyPose(Pose2f(1000.f,1000.f));
//      SpecialAction(SpecialActionRequest::newAction);
//      GoToBallAndKickSide();
//      SearchForBallTogether();
//      KeeperHeadControl();
//      Duel();
//      WalkToBall();
//      if(theRobotInfo.number == 2)
//        Striker();
//      else
//        Stabber();
//      Stand();
//      SpecialAction(SpecialActionRequest::kickBackSoft);
//      KickBallToPose(Vector2f(2000.f,2000.f));
//      WalkToTarget(Pose2f(.4f, 0.f, .7f),Pose2f(90_deg,1200.f,0.f));
//      theHeadControlMode = HeadControl::lookForward;
//      WalkAtSpeedPercentage(Pose2f(-.5f, 0.f, 0.7f));
//      if(state_time < 3000)
//      {
//        theHeadControlMode = HeadControl::lookLeftAndRight;
//        Stand();
//      }
//      else
//      {
//        theHeadControlMode = HeadControl::lookAtBall;
//        WalkToTarget(Pose2f(0.40f, 0.40f, 0.40f),
//                   Pose2f(0, ball.x - inWalkKickForwardX, ball.y + inWalkKickForwardY));
//        WalkToTarget(Pose2f(0.40f, 0.40f, 0.40f),
//                     Pose2f(angleToShoot - 80_deg, ball.x - inWalkKickSideX, ball.y + inWalkKickSideY));
//      }
//      WalkToPose(Pose2f(90_deg, -1500.f, 2500.f));
//      GoToBallAndKick();
//      Dribble2(3000.f, 0.f);
//      DynamicDribble(0.f, 0.f);
//      GoToShootPoint(Vector2f(0,0));
//      AroundBall(Vector2f(0.f, 0.f));
//        theHeadControlMode = HeadControl::lookForward;
//        if(state_time < 3000)
//        Stand();
//        else
//        	TurnOneRound();
    	LookAtBall();
//    	LookLeftAndRight();
//        WalkToPointRunswift(Pose2f(0.8f, 0.8f, 0.8f), Pose2f(0.f, 0.f, 0.f));
    	AlignMent();
    }
  }

  state(test2)
  {
     transition
     {
       if(action_done)
         goto stand;
     }
     action
     {
         theHeadControlMode = HeadControl::lookForward;
        Kick();
     }
  }
  state(dribble)
  {
	  transition
	  {
		  if(ball.positionRobot.norm() > 350.f)
		  {
			  goto start;
		  }
	  }
	  action
	  {
		  DribbleRunswift(true, angleToShoot);
		  LookAtBall();
	  }
  }
  state(kick)
  {
	  transition
	  {
		if(action_done || state_time > 1000)
		    goto start;
	  }
	  action
	  {
		  behavior.supporterOutput.isRequestNotAllowed = true;
		  behavior.supporterOutput.isControlBall = true;
		  theHeadControlMode = HeadControl::lookAtBall;
		  Kick(false);
	  }
  }

//  state(test3)
//  {
//      transition
//      {
//        if(action_done)
//          goto start;
//      }
//      action
//      {
//        theHeadControlMode = HeadControl::lookLeftAndRight;
//          GameFinished();
//        SpecialAction(SpecialActionRequest::keeperBlockLeft,1);
//        Stand();
//        SpecialAction(SpecialActionRequest::sideKickHardLeft);
//      }
//  }

//  state(turnToBall)
//  {
//    transition
//    {
//      if(common.timeSinceBallWasSeen() > 7000)
//        goto searchForBall;
//      if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
//        goto walkToBall;
//    }
//    action
//    {
//      theHeadControlMode = HeadControl::lookForward;
//      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
//    }
//  }

    state(stand)
    {
       transition
       {
//         if(state_time > 2000)
//           goto start;
       }
       action
       {
         theHeadControlMode = HeadControl::lookForward;
         Stand();
       }
    }

//  state(kick)
//  {
//    transition
//    {
//      if(action_done)
//        goto start;
//    }
//    action
//    {
//      theHeadControlMode = HeadControl::lookAtBall;
//      InWalkKick(WalkRequest::right,
//                 Pose2f(goals.angleToGoal, ball.x - inWalkKickForwardX, ball.y + inWalkKickForwardY));
////      InWalkKick(WalkRequest::sidewardsRight,
////                 Pose2f(goals.angleToGoal, ball.x - inWalkKickSideX, ball.y + inWalkKickSideY ));
//    }
//  }

}

