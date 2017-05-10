option(WalkToSupporterDefence)
{
  common_transition
  {

  }

  initial_state(start)
  {
    transition
    {
      goto walkToDefencePosition;
    }
  }

  state(walkToDefencePosition)
  {
    Pose2f speed(.5f, .5f, .5f);
    Pose2f target = supporter.defenderDefencePosition();
//    Pose2f target((ball.global - robot.pose.translation).angle(), 0.5*(ball.global + Vector2f(-4500.f, 0.f)));
    transition
    {
      if(((target - robot.pose).translation.y() < 50 && (target - robot.pose).translation.y() > -50)
                  && ((target - robot.pose).translation.x() < 50 && (target - robot.pose).translation.x() > -50)
                  && ((target - robot.pose).rotation > -0.087f && (target - robot.pose).rotation < 0.087f))
                goto done;
    }
    action
    {
      if(!(ball.wasSeen()|| supporter.useTeamBall) && ball.notSeenTime() > 2000)
        KeeperHeadControl(0, 5000, 5000, 0);
      else if(supporter.ballPositionRelative.x() > 4000)
        KeeperHeadControl(0, 2000, 2000, 2000);
      else if(supporter.ballPositionRelative.x() > 1500)
        KeeperHeadControl(0, 2000, 3000, 3000);
      else
        KeeperHeadControl(0, 0, 2000, 4000);

      if(target.translation.norm() > 1500.f)
        WalkToPose(target);
      else
        WalkToTarget(speed, target - robot.pose);
    }

    DEBUG_DRAWING("representation:Role", "drawingOnField")
    {
      CROSS("representation:Role", target.translation.x(), target.translation.y(), 50, 45, Drawings::solidPen, ColorRGBA(0xff, 0, 0));
    }

  }

  target_state(done)
  {
    Pose2f target((ball.global - robot.pose.translation).angle(), 0.5*(ball.global + Vector2f(-4500.f, 0.f)));
    transition
    {
      if(((target - robot.pose).translation.y() > 100 || (target - robot.pose).translation.y() < -100)
                  || ((target - robot.pose).translation.x() > 100 || (target - robot.pose).translation.x() < -100)
                  || ((target - robot.pose).rotation < -0.154f || (target - robot.pose).rotation > 0.154f))
        goto walkToDefencePosition;
    }
    action
    {
      if(!(ball.wasSeen()|| supporter.useTeamBall) && ball.notSeenTime() > thePenaltyShootParameter.ballNotSeen)
        KeeperHeadControl(0, 5000, 5000, 0);
      else if(supporter.ballPositionRelative.x() > 4000)
        KeeperHeadControl(0, 2000, 2000, 2000);
      else if(supporter.ballPositionRelative.x() > 1500)
        KeeperHeadControl(0, 2000, 3000, 3000);
      else
        KeeperHeadControl(0, 0, 2000, 4000);

      Stand();
    }
  }
}
