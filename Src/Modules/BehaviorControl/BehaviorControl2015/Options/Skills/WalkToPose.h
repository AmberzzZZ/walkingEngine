
option(WalkToPose, (Pose2f) pose, (Pose2f)(Pose2f(5_deg, 100.f, 100.f)) delta, (bool)(false) pause)
{
  const float farDistance = 400.f;
  Pose2f poseRobot = pose - robot.pose;

  float targetX = pose.translation.x();
  float targetY = pose.translation.y();
  float robot2PointAngle = (float)atan2(targetY-robot.y, targetX-robot.x);

  float obsWidth = 250.f;

  Vector2f globalObstacle = Geometry::relative2FieldCoord(Pose2f(robot.rotation,robot.x,robot.y),obstacle.x,obstacle.y);
  Vector2f poseToRobot = robot.pose.translation-pose.translation;
  Vector2f poseToObstacle = globalObstacle-pose.translation;
  float angleDiff = fabs(poseToRobot.angle()-poseToObstacle.angle());

  common_transition
  {
//	  if(fabs(robot.rotation-robot2PointAngle)<common.fromDegrees(5))
//	  {
//		  if(obstacle.x<500 && obstacle.x>5 && fabs(obstacle.y)<obsWidth)
//			  goto avoidObstacle;
//		  if(obstacle.x>=500 && obstacle.x<=1200 && angleDiff<1.0)
//			  goto avoidPath;
//	  }
  }

  initial_state(walkToPoint)
  {
    const Pose2f walkSpeed(0.f, 0.6f, 0.3f);
    transition
    {
      if(common.between(poseRobot.translation.x(), -0.5 * delta.translation.x(), 0.5 * delta.translation.x()) && common.between(poseRobot.translation.y(), -0.5 * delta.translation.y(), 0.5 * delta.translation.y()))
        goto turnToRotation;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRight;
      if(poseRobot.translation.norm() > farDistance)
        WalkToPointWithPath(pose, delta, pause);
      else
      {
        WalkToTarget(walkSpeed, poseRobot);
      }
    }
  }

  state(turnToRotation)
  {
    const Pose2f turnSpeed(0.6f, 0.f, 0.f);
    transition
    {
      if(!common.between(poseRobot.translation.x(), -delta.translation.x(), delta.translation.x()) || !common.between(poseRobot.translation.y(), -delta.translation.y(), delta.translation.y()))
        goto walkToPoint;
      if(common.between(poseRobot.rotation, -delta.rotation, delta.rotation))
        goto holdOn;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRight;
      WalkToTarget(turnSpeed, poseRobot);
    }
  }

  state(avoidObstacle)
  {
 	  transition
 	  {
 		  if(action_done)
 			  goto walkToPoint;
 	  }
 	  action
 	  {
 		 theHeadControlMode = HeadControl::lookLeftAndRight;
 		 AvoidObstacle(pose.translation);
 	  }
  }

   state(avoidPath)
   {
	   transition
	   {
		   if(action_done)
			   goto walkToPoint;
	   }
	   action
	   {
		   AvoidPath(pose);
	   }
	}

  target_state(holdOn)
  {
    const Pose2f speed(0.3f, 0.3f, 0.3f);
    transition
    {
      if(!common.between(poseRobot.translation.x(), -delta.translation.x(), delta.translation.x()) || !common.between(poseRobot.translation.y(), -delta.translation.y(), delta.translation.y()))
        goto walkToPoint;
      if(!common.between(poseRobot.rotation, -delta.rotation, delta.rotation))
        goto turnToRotation;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRight;
      if(pause)
        Stand();
      else
        WalkToTarget(speed, poseRobot);
    }
  }

};
