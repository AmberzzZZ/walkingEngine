/*
 * @File:   Dribble.h
 * @Author: nh
 *@Usage: 通过输入目标前全局坐标，可将球带到目标点为圆心，半径20cm的圆内
 * @Date: 2017.3.18
 */

option(Dribble, (float) x, (float) y)
{
  static float leftOrRightKick;
  common_transition
  {
    if(ball.distance > 600 && fabs(ball.angleDeg) > 30.f)
      goto turnToBallWhileWalkToBall;
//    if (ball.notSeenTime() > 3000)
//      goto searchForBall;
  }

  initial_state(stand)
  {
    transition
    {
//      if (state_time > 0)   //@XHan: 400 #back-up
        goto walkToBall;
    }
    action
    {
//      Stand();
    }
  }

  state(walkToBall)
  {
    transition
    {
      if (ball.distance < 600.f
          && (theRobotPose.inverse() * Vector2f(x,y)).angle() < 0)
        goto goFromLeftsideBehindBall;
      if (ball.distance < 600.f
          && (theRobotPose.inverse() * Vector2f(x,y)).angle() > 0)
//                goto alignToGoal;//(goto alighnToGoal).bak @dmx
        goto goFromRightsideBehindBall;
//      std::cout<<"ballangle--->"<<ball.angleDeg<<std::endl;
//      if (fabs(ball.angleDeg) > 10.f)
//        goto turnToBallWhileWalkToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftOrRightWhileDribble;
      if(ball.distance > 500.f)
         WalkToTarget(Pose2f(0.1f, 0.8f, 0.f), Pose2f(ball.angleRad, ball.x, 0));
      else
         WalkToTarget(Pose2f(0.1f, 0.5f, .2f), Pose2f(ball.angleRad, ball.x, ball.y));
    }
  }

  state(goFromLeftsideBehindBall)
  {
    transition
    {
      if (common.between((theRobotPose.inverse() * Vector2f(x, y)).angle(),
                         common.fromDegrees(-110), common.fromDegrees(110)))
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2f(.7f, .6f, 0.f),
                   Pose2f(ball.angleRad + (float)(atan2(400.f, ball.distance)),
                          ball.distance, 0));
    }
  }

  state(goFromRightsideBehindBall)
  {
    transition
    {
      if (common.between((theRobotPose.inverse() * Vector2f(x, y)).angle(),
                         common.fromDegrees(-110), common.fromDegrees(110)))
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2f(.7f, .6f, 0.f),
                   Pose2f(ball.angleRad - float(atan2(400.f, ball.distance)),
                          ball.distance, 0));
    }
  }

  state(turnToBall)
  {
    transition
    {
      if(fabs(ball.angleRad)<common.fromDegrees(10))
        goto alignBehindBall;
    }
    action
    {
      WalkToTarget(Pose2f(0.5f, 0.0f, 0.0f), Pose2f(ball.angleRad, 0.f, 0.f));
    }
  }

  state(alignBehindBall)
  {
    transition
    {
      if (common.between((theRobotPose.inverse() * Vector2f(x, y)).angle(),
                         common.fromDegrees(-5), common.fromDegrees(5)))
      {
        ///               leftOrRightKick=(ball.y>0?43:43);
        //            leftOrRightKick=0;//中間帶球參數
        goto dribble;
      }
    }
    action
    {
    
      theHeadControlMode = HeadControl::lookLeftOrRightWhileDribble;
     // if(ball.distance>500)AlignMent(Vector2f(x,y));
     // else GoBehindBall(x, y);
     // GoBehindBall(x, y);
      AlignMent(Vector2f(x,y));
     // if(ball.distance<250)
     // {
     //   if((theRobotPose.inverse()*Vector2f(x,y)).angle() > 0)  ///move counterclockwise
     //   {
     //     if(ball.angleRad>0)
     //       WalkToTarget(Pose2f(.3f, .7f, .7f),
     //                    Pose2f(ball.angleRad, ball.x-160, ball.y-40));
     //     else
     //       WalkToTarget(Pose2f(.3f, .7f, .7f),
     //                    Pose2f(ball.angleRad, ball.x-160, ball.y-40));
     //   }
     //   else
     //   {
     //     if(ball.angleRad<0)
     //       WalkToTarget(Pose2f(.3f, .7f, .7f),
     //                    Pose2f(ball.angleRad, ball.x-160, ball.y+40));
     //     else
     //       WalkToTarget(Pose2f(.3f, .7f, .7f),
     //                    Pose2f(ball.angleRad, ball.x-160, ball.y+40));
     //   }
     // }
     // else
     // {
     //   WalkToTarget(Pose2f(.5f, 0.7f, 0.7f), theBallModel.estimate.position);
     // }
    }
  }

  state(dribble)
  {
    transition
    {
      if (fabs((theRobotPose.inverse() * Vector2f(x, y)).angle())
          > common.fromDegrees(20.f) || fabs(ball.angleDeg) > 45.f)
        goto turnToBall;
//            if (state_time>3000)
//                goto turnLittleAngle;
      if (fabs(ball.global.x() - x) < 150
          && fabs(ball.global.y() - y) < 150 && ball.distance < 200)
        goto finish;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftOrRightWhileDribble;
      if(ball.y > -4)  //BALL AT LEFT
        leftOrRightKick = 38;
      else if(ball.y < -6)  //BALL AT RIGHT
        leftOrRightKick = -42;
      // leftOrRightKick=0;
//           leftOrRightKick = (ball.y > -4 ? 38 : -42);///设置球的偏离量，左右脚都可带球
//      if(fabs((ball.y - leftOrRightKick)) < 10.f)
//    	  WalkAtSpeedPercentage(Pose2f(0.f, 0.7f, 0.f));
      if (ball.distance > 300 || fabs(ball.y - leftOrRightKick) > 15.f)
        WalkToTarget(Pose2f(.5f, .7f, .5f),
                     Pose2f((theRobotPose.inverse() * Vector2f(x, y)).angle(),
                            ball.x - 50, ball.y - leftOrRightKick));
      else
    	  WalkAtSpeedPercentage(Pose2f(0.f, 0.5f, 0.f));
    //WalkToTarget(Pose2f(.5f, .7f, .5f), Pose2f(0,ball.x-50,ball.y-leftOrRightKick));
    }
  }

  target_state(finish)
  {
    transition
    {
    }
    action
    {
      theHeadControlMode=HeadControl::lookLeftAndRight;
      Stand();
    }
  }

  state(searchForBall)
  {
    transition
    {
      if (ball.notSeenTime() < 300)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkAtSpeedPercentage(Pose2f(1.f, 0.f, 0.f));
    }
  }

  state(turnToBallWhileWalkToBall)
  {
    transition
    {
//      if (ball.notSeenTime() > 3000)
//        goto searchForBall;
      if (fabs(ball.angleRad) < common.fromDegrees(5.f) && state_time > 500)
        goto walkToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(ball.angleRad, 0.f, 0.f));
    }
  }

}