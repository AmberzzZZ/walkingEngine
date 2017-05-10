/*
 * File:   GoBehindBall2.h
 * Author: yongqi
 * Created on 2016.04.23
 */
option(GoBehindBall2, (float) (4500) x, (float) (0) y)
{
  common_transition
  {

  }


  initial_state(walkToBall)
  {
    transition
    {
      if(action_done || (fabs(ball.angleDeg) < 10. && ball.distance < 500.f && ball.distance > 200.f))
      {
        if(ball.global.y() > robot.y)
          goto goFromLeftsideBehindBall;
        else
          goto goFromRightsideBehindBall;
//          goto alignBehindBall;
      }
    }
    action
    {
//      theHeadControlMode = HeadControl::lookLeftAndRight;
//      WalkToTarget(Pose2f(0.f, 0.7f, 0.f), theBallModel.estimate.position);
      WalkToBall(450.f);
    }
  }

  state(goFromLeftsideBehindBall)
  {
    transition
    {
      if(common.between((theRobotPose.inverse() * Vector2f(x, y)).angle(),
                        -110_deg, 110_deg)
          && ball.global.x() > robot.x)
	      goto turnToBall;
      if(ball.distance > 500)
        goto walkToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
        WalkToTarget(Pose2f(.5f, .5f, 0.f),
                   Pose2f(ball.angleRad + 90_deg - 15_deg, 15_deg * ball.distance, 0.f));
    }
  }

  state(goFromRightsideBehindBall)
  {
    transition
    {
      if(common.between((theRobotPose.inverse() * Vector2f(x, y)).angle(),
                         -110_deg, 110_deg)
          && ball.global.x() > robot.x)
	      goto turnToBall;
      if(ball.distance > 500)
        goto walkToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2f(.5f, .5f, 0.f),
                    Pose2f(ball.angleRad - 90_deg + 15_deg, 15_deg * ball.distance, 0));
    }
  }

  state(turnToBall)
  {
    transition
    {
      if(fabs(ball.angleRad) < common.fromDegrees(10))
      	goto alignBehindBall;
    }
    action
    {
      WalkToTarget(Pose2f(0.8f, 0.f, 0.f), Pose2f(ball.angleRad, 0.f, 0.f));
    }
  }


//  state(turnToBallWhileWalkToBall)
//  {
//    transition
//    {
//      if(ball.notSeenTime() > 3000)
//        goto searchForBall;
//      if(std::abs(ball.angleDeg) < 5. && state_time > 500)
//	      goto walkToBall;
//    }
//    action
//    {
//      theHeadControlMode = HeadControl::lookForward;
//      WalkToTarget (Pose2f(0.5f, 0.5f, 0.5f), Pose2f(ball.angleRad, 0.f, 0.f));
//    }
//  }

  state (alignBehindBall)
  {
    transition
    {
      if(ball.distance < 500 && fabs(ball.angleDeg) < 10. &&
          fabs((theRobotPose.inverse() * Vector2f(x, y)).angle()) < common.fromDegrees(10))
      {
	      goto finish;
      }
      else if(ball.distance > 500)
      {
        goto walkToBall;
      }
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      if (ball.distance < 400)
      {
        if ((theRobotPose.inverse() * Vector2f (x, y)).angle() > 0)  ///move counterclockwise
        {
          if (ball.angleRad > 0)
            WalkToTarget (Pose2f(.8f, .9f, .7f),
                          Pose2f(ball.angleRad, ball.positionRobot.x() - 160, ball.positionRobot.y() - 40));
         else
            WalkToTarget (Pose2f(.8f, .9f, .7f),
                          Pose2f(0, ball.positionRobot.x() - 160, ball.positionRobot.y() - 40));
        }
        else
        {
          if (ball.angleRad < 0)
            WalkToTarget (Pose2f(.8f, .9f, .7f),
                          Pose2f(ball.angleRad, ball.positionRobot.x() - 160, ball.positionRobot.y() + 40));
          else
            WalkToTarget (Pose2f(.8f, .9f, .7f),
                          Pose2f(0, ball.positionRobot.x() - 160, ball.positionRobot.y() + 40));
        }
      }
      else
      {
//                 theHeadControlMode = HeadControl::lookLeftOrRightWhileDribble;
        WalkToTarget (Pose2f(0.f, 0.7f, 0.f), theBallModel.estimate.position);
      }
    }
  }

  target_state(finish)
  {
    transition
    {
      if(ball.distance > 500 || fabs(ball.angleDeg) > 11.f ||
          fabs((theRobotPose.inverse() * Vector2f(x, y)).angle()) > common.fromDegrees(11))
      {
        goto walkToBall;
      }
    }
    action
    {
//      theHeadControlMode = HeadControl::lookLeftAndRight;
      theHeadControlMode = (state_time % 4000) < 2000 ?
          HeadControl::lookLeftAndRight : HeadControl::lookAtBall;
      Stand();
    }
  }
}
