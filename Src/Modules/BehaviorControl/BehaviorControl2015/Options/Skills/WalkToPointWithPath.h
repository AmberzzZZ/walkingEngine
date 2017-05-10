/**
 * @file WalkToPoint.h
 * @author yongqi Lee
 * @date Jun 4, 2016
 * @Usage walk to globe point
 * @param point: target x, y, rotation
 *
 */

option(WalkToPointWithPath, (Pose2f) point, (Pose2f)(Pose2f(8_deg, 100.f, 100.f)) tolerate, (bool)(false) pause)
{
  float &x = point.translation.x();
  float &y = point.translation.y();
//  Angle &r = point.rotation;
  float &tolerateX = tolerate.translation.x();
  float &tolerateY = tolerate.translation.y();
  Angle &tolerateRad = tolerate.rotation;
  const float farAwayDistance = 500.f;
  const Angle farAwayRad = 30_deg;
  const Angle nearbyRad = 45_deg;
  Vector2f relative = theRobotPose.inverse() * point.translation;
  float distance = relative.norm();
  Angle angle = relative.angle();

  common_transition
  {
    relative = theRobotPose.inverse() * point.translation;
    distance = relative.norm();
    angle = relative.angle();
    if(fabs(robot.x - x) < tolerateX && fabs(robot.y - y) < tolerateY)
      goto finish;
  }

  initial_state(judge)
  {
    transition
    {
       if ((distance > farAwayDistance && fabs(angle) > tolerateRad))
       {

       }
// //          || (distance <= farAwayDistance && fabs(angle) > nearbyRad))
//         goto turnToPoint;
       else
        goto walkStraightToPoint;
    }
    action
    {
    	WalkToTarget(Pose2f(0.8f, 0.f, 0.f),
    	                   Pose2f(angle, 0.f, 0.f));
    }
  }

  state(turnToPoint)
  {
    transition
    {
      if(fabs(angle) < tolerateRad)
        goto walkStraightToPoint;
    }
    action
    {
      WalkToTarget(Pose2f(0.8f, 0.f, 0.f),
                   Pose2f(angle, 0.f, 0.f));
    }
  }

  state(walkStraightToPoint)
  {
    transition
    {
      if(fabs(robot.x - x) < tolerateX && fabs(robot.y - y) < tolerateY)
        goto finish;
      // if((distance > farAwayDistance && fabs(angle) > farAwayRad)
      //     || (distance <= farAwayDistance && fabs(angle) > nearbyRad))
      //   goto turnToPoint;
    }
    action    //@YY
    {
//       if(distance > farAwayDistance)
// //        WalkToTarget(Pose2f(0.1f, 1.f, 0.f), Pose2f(angle, relative.x(), 0));
//         WalkAtSpeedPercentage(Pose2f(0.f, .8f, 0.f));
//       else
        // WalkToTarget(Pose2f(0.f, 0.5f, .4f), Pose2f(0.f, relative.x(), relative.y()));
          WalkToTargetWithPath(Pose2f(0.3f, 0.6f, 0.3f), Pose2f(0.f, point.translation.x(), point.translation.y())); //@YY
    }
  }

//  state(turnToRotation)
//  {
//    transition
//    {
//      if(fabs(robot.rotation - r) < tolerateRad)
//        goto finished;
//    }
//    action
//    {
//
//      WalkToTarget(Pose2f(0.5f, 0.f, 0.f),
//                   Pose2f(r-robot.rotation, 0.f, 0.f));
//    }
//  }

  target_state(finish)
  {
    transition
    {
      if(!(fabs(robot.x - x) < tolerateX && fabs(robot.y - y) < tolerateY)
          && state_time > 1000)
        goto judge;
    }
    action
    {
      if(pause)
        Stand();
      else
        WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.f));
    }
  }
}
