option(WalkToTarget, (const Pose2f&) speed, (const Pose2f&) target)
{
  const float shortDistance = 200.f;

  float distance = target.translation.norm();
  if(distance)

  float targetRadian = atan2(target.translation.y(), target.translation.x());
  initial_state(turnToTarget)
  {
    transition
    {
      if(theWalkingEngineOutput.actionDone && state_time > 200)
        goto walkToTarget;
    }
    action
    {
      WalkToTarget2(Pose2f(speed.rotation, 0.f, 0.f), Pose2f(targetRadian, 0.f, 0.f));
    }
  }

  state(walkToTarget)
  {
    transition
    {
      if(theWalkingEngineOutput.actionDone && state_time > 200)
        goto turn;
    }
    action
    {
      WalkToTarget2(Pose2f(0.f, speed.translation.x() * 300, 0.f), Pose2f(0.f, target.translation.x(), 0.f));
    }
  }

  target_state(turn)
  {
    action
    {
      WalkToTarget2(Pose2f(speed.rotation, 0.f, 0.f), Pose2f(target.rotation - targetRadian, 0.f, 0.f));
    }
  }

}
