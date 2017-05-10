/** behavior for the ready state */
option(ReadyState)
{
  common_transition
  {
    if(theRole.role == Role::keeper)
      goto keeperReady;
    else if(theRole.role == Role::striker)
      goto strikerReady;
    else if(theRole.role == Role::supporter)
      goto supporterReady;
    else if(theRole.role == Role::defender)
      goto defenderReady;
    else if(theRole.role == Role::stabber)
      goto stabberReady;
    else
      goto stand;
  }

  initial_state(stand)
  {
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }

  state(keeperReady)
  {
    transition
    {

    }

    action
    {
      theHeadControlMode = HeadControl::lookForward;
      GoToReadyPose(robot.keeperReadyPose);
    }
  }

  state(strikerReady)
  {
    transition
    {

    }

    action
    {
      theHeadControlMode = HeadControl::lookForward;
      if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
        GoToReadyPose(robot.strikerReadyPoseWhenKickOff);
      else
        GoToReadyPose(robot.strikerReadyPose);
    }
  }

  state(supporterReady)
  {
    transition
    {

    }

    action
    {
      theHeadControlMode = HeadControl::lookForward;
      if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
        GoToReadyPose(robot.supporterReadyPoseWhenKickOff);
      else
      {
        GoToReadyPose(robot.supporterReadyPose);
      }
    }
  }

  state(defenderReady)
  {
    transition
    {

    }

    action
    {
      theHeadControlMode = HeadControl::lookForward;
      GoToReadyPose(robot.defenderReadyPose);
    }
  }

  state(stabberReady)
  {
    transition
    {

    }

    action
    {
      theHeadControlMode = HeadControl::lookForward;
      GoToReadyPose(robot.stabberReadyPose);
    }
  }

}
