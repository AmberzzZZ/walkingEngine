option(AvoidShootPath)
{
  float rotationBallToRobot = (ball.global - robot.pose.translation).angle();
  float rotationBallToGoal = (ball.global - Vector2f(4500.f, 0.f)).angle();
  float delta = rotationBallToRobot - rotationBallToGoal;
  initial_state(start)
  {
    transition
    {
      if(fabs(delta) > 20_deg)
        goto turnToBall;
      if(delta > 0)
        goto avoidLeft;
      else
        goto avoidRight;
    }
  }

  state(avoidLeft)
  {
    transition
    {
      if(state_time > 2000 && fabs(delta) > 20_deg)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      if(fabs(robot.rotation) < 45_deg)
      {
//        std::cout<<"side------------------l"<<std::endl;
        WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.8f));
      }
      else if(fabs(robot.rotation) > 135_deg)
      {
//        std::cout<<"side------------------r"<<std::endl;
        WalkAtSpeedPercentage(Pose2f(0.f, 0.f, -0.8f));
      }
      else if(robot.rotation > 0_deg)
      {
//        std::cout<<"forward------------------f"<<std::endl;
        WalkAtSpeedPercentage(Pose2f(0.f, 0.8f, 0.f));
      }
      else
      {
//        std::cout<<"forward------------------b"<<std::endl;
        WalkAtSpeedPercentage(Pose2f(0.f, -0.8f, 0.f));
      }
    }
  }

  state(avoidRight)
  {
    transition
    {
      if(state_time > 2000 && fabs(delta) > 20_deg)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      if(fabs(robot.rotation) < 45_deg)
      {
//        std::cout<<"side------------------r"<<std::endl;
        WalkAtSpeedPercentage(Pose2f(0.f, 0.f, -0.8f));
      }
      else if(fabs(robot.rotation) > 135_deg)
      {
//        std::cout<<"side------------------l"<<std::endl;
        WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.8f));
      }
      else if(robot.rotation > 0_deg)
      {
//        std::cout<<"forward------------------b"<<std::endl;
        WalkAtSpeedPercentage(Pose2f(0.f, -0.8f, 0.f));
      }
      else
      {
//        std::cout<<"forward------------------f"<<std::endl;
        WalkAtSpeedPercentage(Pose2f(0.f, 0.8f, 0.f));
      }
    }
  }


  target_state(turnToBall)
  {
    transition
    {
      if(fabs(delta) < 15_deg)
        goto start;
    }
    action
    {
      TurnToBall();
    }
  }

}
