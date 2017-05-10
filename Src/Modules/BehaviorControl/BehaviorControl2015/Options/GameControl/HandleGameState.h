/** Triggers the options for the different game states.
 *  This option also invokes the get up behavior after a fall, as this is needed in most game states.
 */
option(HandleGameState)
{
  /** As game state changes are discrete external events and all states are independent of each other,
      a common transition can be used here. */
  common_transition
  {
    if(theGameInfo.state == STATE_INITIAL)
      goto initial;
    else if(theGameInfo.state == STATE_FINISHED)
      goto finished;
//    else if(theFallDownState.state != FallDownState::upright && theFallDownState.state != FallDownState::undefined)
    else if(theFallDownState.state == FallDownState::onGround)
      goto getUp;
    else if(theGameInfo.state == STATE_READY)
      goto ready;
    else if(theGameInfo.state == STATE_SET)
      goto set;
    else if(theGameInfo.state == STATE_PLAYING)
      goto playing;
  }

  /** Stand still and wait. */
  initial_state(initial)
  {
    action
    {
      theHeadControlMode = HeadControl::none;
      SetHeadPanTilt(0.f, 0.f, 150_deg);
      if(theGameInfo.secondaryState == STATE2_PENALTYSHOOT)
        PlaySound(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber
                  ? "penaltyStriker.wav" : "penaltyKeeper.wav");
//      Stand();
      SpecialAction(SpecialActionRequest::standHigh);
    }
  }

  /** Stand still and wait. If somebody wants to implement cheering moves => This is the place. ;-) */
  state(finished)
  {
    action
    {
      theHeadControlMode = HeadControl::lookForward;
//      Stand();
      SpecialAction(SpecialActionRequest::sitDown);
    }
  }

  /** Get up from the carpet. */
  state(getUp)
  {
    action
    {
      Annotation("Getting up.");
//      GetUp();
      StandUp();
    }
  }

  /** Walk to kickoff position. */
  state(ready)
  {
    action
    {
//      ArmContact();
      ReadyState();
    }
  }

  /** Stand and look around. */
  state(set)
  {
    action
    {
      if(theRole.role == Role::supporter)
      {

        // detect opponent robots
        Vector2f oppRobotPosition = Geometry::relative2FieldCoord(robot.pose,obstacle.center);
//        std::cout<<"oppRobotPosition.y()--> "<<oppRobotPosition.y()<<std::endl;
        if(oppRobotPosition.x() > 300.f && fabs(oppRobotPosition.y()) < 300)
        {
          if(oppRobotPosition.y() > 0)
          {
            behavior.supporterOutput.kickOffDirection = SupporterBehavior::RIGHT;
          }
          else
          {
            behavior.supporterOutput.kickOffDirection = SupporterBehavior::LEFT;
          }
        }
        else
        {
          behavior.supporterOutput.kickOffDirection = SupporterBehavior::MIDDLE;
        }
      }
      if(theRole.role == Role::keeper || theRole.role == Role::defender)
      {
    	  theHeadControlMode = HeadControl::lookLeftAndRightFast;
      }
      else
      {
    	  if(ball.notSeenTime() > 2000.f && !ball.isTeamPositionValid)
			  theHeadControlMode = HeadControl::lookLeftAndRightFast;
		  else
			  theHeadControlMode = HeadControl::lookAtBall;
      }


      Stand();
//      SpecialAction(SpecialActionRequest::standHigh);
    }
  }

  /** Play soccer! */
  state(playing)
  {
    action
    {
//      PlaySound("yah.wav");
//      if(theRobotInfo.number != 1)
//      {
//        AvoidObstacleNearby();
//      }
//      ArmContact();
      PlayingState();
    }
  }
}
