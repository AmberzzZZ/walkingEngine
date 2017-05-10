/*
 *@author: Xincheng Han
 *@Usage: GoToReadyPose(theReadyPose.pose)
 *@param: pose is stored in originalPose.cfg according to roles
 */

option(GoToReadyPose, (Pose2f) pose)
{

//  Pose2f readyPose(pose);

  float avoidDistance = 0.6f;	//Distance threshold for not avoiding obstacle near the target pose
  float obsWidth = 670.f;
  if((sqr(robot.x-pose.translation.x())+sqr(robot.y-pose.translation.y()))<sqr(1000))
    obsWidth = 500.f;
  if(fabs(Geometry::fieldCoord2Relative(theRobotPose,Vector2f(pose.translation.x(),pose.translation.y())).x()-obstacle.x)<400)
    obsWidth = 300;
#ifdef TARGET_SIM
  obsWidth = 330.f;
  if((sqr(robot.x-pose.translation.x())+sqr(robot.y-pose.translation.y()))<sqr(1000))//1200 for debug
    obsWidth = 250.f;
  if(fabs(Geometry::fieldCoord2Relative(theRobotPose,Vector2f(pose.translation.x(),pose.translation.y())).x()-obstacle.x)<400)
    obsWidth = 150;
#endif
  common_transition
  {
//    if((theRobotPose.translation/1000-pose.translation/1000).norm() > avoidDistance)
//    {
//      if(obstacle.x<500 && obstacle.x>100 && fabs(obstacle.y)<obsWidth)
//        goto avoidObstacle;
////			if(obstacle.x>500 && obstacle.x<1000 && fabs(obstacle.y)<obsWidth &&\
////					(pose.translation-Geometry::relative2FieldCoord(Pose2f(robot.rotation,robot.x,robot.y),\
////							obstacle.x,obstacle.y)).norm()>800)
////				goto avoidPath;
//    }
  }

  initial_state(walkToPose)
  {
    transition
    {
      if(action_done)
      {
        goto ready;
      }
    }
    action
    {
    	theHeadControlMode = HeadControl::lookLeftAndRight;
      WalkToPose(pose, Pose2f(4_deg, 50.f, 50.f), true);
    }
  }

  state(avoidObstacle)
   {
     transition
     {
       if((theRobotPose.translation/1000-pose.translation/1000).norm() < avoidDistance/2)
         goto turnToPose;
       if(action_done)
         goto walkToPose;
     }

     action
     {
       AvoidObstacle(pose.translation);
     }
   }

//  state(learn)
//  {
//    transition
//    {
//      if(state_time > 400)
//        goto turnToPose;
//    }
//    action
//    {
//      Stand();
//      theHeadControlMode = HeadControl::lookForward;
//      robot.setLearning(true);
//    }
//  }

//  state(turnToPose)
//  {
//    transition
//    {
//      if(action_done)
//      {
//        goto ready;
//      }
//    }
//    action
//    {
//      WalkToPose(pose, 20, 20, 20, 20, 3, 1);
//    }
//  }

    state(turnToPose)
    {
      transition
      {
        if(action_done)
        {
          goto ready;
        }
      }
      action
      {
    	  theHeadControlMode = HeadControl::lookLeftAndRight;
        WalkToTarget(Pose2f(.6f, 0.f, 0.f), Pose2f(robot.rotation, 0.f, 0.f));
      }
    }

  target_state(ready)
  {
    transition
    {
        if((pose.translation - robot.pose.translation).norm() > 1000 || robot.pose.translation.x() > 0.f)
            goto walkToPose;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRight;
//      Stand();
      SpecialAction(SpecialActionRequest::standHigh);
    }
  }
}
