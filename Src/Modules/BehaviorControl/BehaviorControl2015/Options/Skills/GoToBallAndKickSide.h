option (GoToBallAndKickSide)
{
  bool useRightFoot = false;
  const float kickX = 180.f;
  const float kickY = 40.f;

  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRight;
      Stand();
    }

  }

  state(turnToBall)
  {
    transition
    {
      if(std::abs (theBallModel.estimate.position.angle()) < 5.f / 180 * pi)
        goto walkToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2f(.3f, .5f, .5f),
                   Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
    }
  }

  state(walkToBall)
  {
    transition
    {
      if(theBallModel.estimate.position.norm() < 500.f)
        goto alignToGoal;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2f(.1f, .5f, .5f), theBallModel.estimate.position);
    }
  }

  state(alignToGoal)
  {
    transition
    {
      if(common.between(theBallModel.estimate.position.y(), -kickY -10.f, -kickY + 10.f)
          && common.between(theBallModel.estimate.position.x(), kickX -10.f, kickX+5.f)
          && (!useRightFoot))
      {
          goto kick; //using left foot
      }
      else if(common.between(theBallModel.estimate.position.y(), kickY -10.f, kickY +10.f)
               && common.between(theBallModel.estimate.position.x(), kickX -10.f, kickX+5.f)
               && (useRightFoot))
      {
        goto kick; //using right foot
      }
    }
    action
    {
//      theHeadControlMode = (state_time % 4000) < 2000 ? HeadControl::lookLeftAndRight : HeadControl::lookAtBall;
      theHeadControlMode = HeadControl::lookAtBall;
      if(!useRightFoot)
      WalkToTarget(Pose2f(.3f, .5f, .5f),
        Pose2f(0.f, theBallModel.estimate.position.x () - kickX,
               theBallModel.estimate.position.y () + kickY));
      else
        WalkToTarget (Pose2f (.3f, .5f, .5f),
        Pose2f (0.f, theBallModel.estimate.position.x () - kickX,
                theBallModel.estimate.position.y () - kickY));
    }
  }

  state(kick)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      if(useRightFoot)
      {
        SpecialAction(SpecialActionRequest::sideKickHardRight);
      }
      else
      {
        SpecialAction(SpecialActionRequest::sideKickHardLeft);
      }
    }
  }

}
