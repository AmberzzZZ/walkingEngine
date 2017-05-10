/**
 * @File Duel.h
 * @Usage fast kick the ball without aligning to the goal
 * @author Hanbin Wang
 * @Date March 3rd, 2014
 */
option(Duel)
{
  //During the InWalkKick, the distance between the foot and ball
  const float kickX = 150.f;
  const float ballDisThreshold = 500.f;   //@XHan:800#back-up
  const float obstacleDisThresholdX = 800.f;
  bool kickLeft = false;
  bool kickRight = true;

  common_transition
  {
//      std::cout<<"ball.isNearOppGoalLeft()---> "<<ball.isNearOppGoalLeft()<<std::endl;
//      std::cout<<"ball.isNearOppGoalRight()---> "<<ball.isNearOppGoalRight()<<std::endl;
//      std::cout<<"libCodeRelease.kickLeft "<<libCodeRelease.kickLeft<<std::endl;
//      std::cout<<"libCodeRelease.kickRight "<<libCodeRelease.kickRight<<std::endl;
//      std::cout<<"libCodeRelease.distance----> "<<libCodeRelease.distance<<std::endl;
//      std::cout<<"obstacleDisThresholdX----> "<<obstacleDisThresholdX<<std::endl;
//      std::cout<<"libCodeRelease.y----> "<<libCodeRelease.y<<std::endl;

//    if(ball.isNearOppGoalLeft() || (obstacle.distance < obstacleDisThresholdX
//        && obstacle.y >= 0 && obstacle.y < 500))
    if(obstacle.distance < obstacleDisThresholdX && obstacle.y >= 0 && obstacle.y < 500)
    {
      kickRight = true;
      kickLeft = false;
    }
//    else if(ball.isNearOppGoalRight() || (obstacle.distance<obstacleDisThresholdX
//        && obstacle.y < 0 && obstacle.y > -500))
    else if(obstacle.distance < obstacleDisThresholdX && obstacle.y < 0 && obstacle.y > 500)
    {
      kickLeft = true;
      kickRight = false;
    }
    else
    {
      kickLeft = false;
      kickRight = false;
    }
    //std::cout<<"duel"<<std::endl;
  }

  state(start)
  {
    transition
    {
      if(ball.distance < 300.f && ball.global.x() > robot.x
          && obstacle.distance > 200)
        goto alignToGoal;
      else if(ball.distance > 300.f)
        goto walkToBall;
      //if(state_time>3000)
      // goto debugState;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRight;
      Stand();
    }
  }

  initial_state(walkToBall)
  {
    transition
    {
      if(action_done && ball.global.x() > robot.x)
        goto alignToGoal;
      else if((action_done || ball.distance < 650.f) && ball.global.x() < robot.x)
        goto kickBack;
//      else if(fabs(ball.angleDeg) > 10)
//        goto turnToBall;
    }
    action
    {
      WalkToBall(200.f);     //@XHan:To be tested
    }
  }

  state(kick)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;
      if(!(kickLeft||kickRight))
        goto alignToGoal;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      if(kickLeft&&(!kickRight))
      {
        InWalkKick(WalkRequest::sidewardsRight,
                   Pose2f(goals.angleToGoal, ball.x - kickX, ball.y));

      }
      else if((!kickLeft)&&(kickRight))
      {
        InWalkKick(WalkRequest::sidewardsLeft,
                   Pose2f(goals.angleToGoal, ball.x - kickX, ball.y ));
      }
    }
  }

  state(kickBack)
  {
    transition
    {
      if(ball.global.x() > robot.x + 200.f)
        goto walkToBall;
//      else if(ball.distance > 800.f)
//        goto walkToBall;
//      else if(action_done)
//        goto start;
    }
    action
    {
//      if(ball.global.x() > 3000)
        GoSideAndKickBack(4500,0);
//      else
//        GoSideAndKickBack(2500,0);
    }
  }

//  state(turnToBall)
//  {
//    transition
//    {
//      if(action_done)
//        goto walkToBall;
//    }
//    action
//    {
//      TurnToBall();
//    }
//  }

  state(alignToGoal)
  {
    transition
    {
      if(ball.distance > ballDisThreshold || fabs(ball.angleDeg) > 90.f)
        goto walkToBall;
      if(!(kickLeft||kickRight) && ball.positionRobot.y() > 0 && ball.positionRobot.y() < 100.f
          && fabs(goals.angleToGoal) < common.fromDegrees(10.f))
        goto alignLeft;
      else if(!(kickLeft||kickRight) && ball.positionRobot.y() > -100.f && ball.positionRobot.y() < 0
          && fabs(goals.angleToGoal) < common.fromDegrees(10.f))
        goto alignRight;
      else if(fabs(goals.angleToGoal) < common.fromDegrees(10.f)
          && fabs(ball.positionRobot.y()) < 100.f)
        goto alignBehindBall;
    }
    action
    {
      theHeadControlMode = (state_time % 4000 < 2000) ?
          HeadControl::lookAtBall : HeadControl::lookLeftAndRight;
      WalkToTarget(Pose2f(0.7f, 0.7f, 0.7f),
                   Pose2f(goals.angleToGoal, ball.positionRobot.x()-250.f, ball.positionRobot.y()));
    }
  }

  state(alignLeft)
  {
    transition
    {
//      if(fabs(obstacle.y) < 200.f)  //@XHan:add this condition
//        goto alignToGoal;
      if(common.between(ball.positionRobot.y(), 40.f, 65.f)
          && common.between(ball.positionRobot.x(), kickX - 10.f, kickX + 20.f)
          && fabs(goals.angleToGoal) < common.fromDegrees(3.f))
        goto kickLeft;
      if (ball.distance > ballDisThreshold || fabs(ball.angleDeg) > 90)
        goto walkToBall;
    }
    action
    {
      //theHeadControlMode = HeadControl::lookAtBall;
      theHeadControlMode = (state_time%4000<2000) ?
          HeadControl::lookAtBall : HeadControl::lookLeftAndRight;
      WalkToTarget(Pose2f(0.65f, 0.65f, 0.65f),
                   Pose2f(goals.angleToGoal, ball.positionRobot.x() - kickX, ball.positionRobot.y()-50));
    }
  }

  state(alignRight)
  {
    transition
    {
//      if(fabs(obstacle.y)<200.f)  //@XHan:add this condition
//        goto alignToGoal;
      if(common.between(ball.positionRobot.y(), -65.f, -40.f)
          && common.between(ball.positionRobot.x(), kickX - 10.f, kickX + 20.f)
          && fabs(goals.angleToGoal) < common.fromDegrees(3.f))
        goto kickRight;
      if (ball.distance > ballDisThreshold || fabs(ball.angleDeg) > 90)
        goto walkToBall;
    }
    action
    {
      //theHeadControlMode = HeadControl::lookAtBall;
      theHeadControlMode = (state_time%4000<2000) ?
          HeadControl::lookAtBall : HeadControl::lookLeftAndRight;
      WalkToTarget(Pose2f(0.65f, 0.65f, 0.65f),
                   Pose2f(goals.angleToGoal, ball.positionRobot.x() - kickX, ball.positionRobot.y()+50));
    }
  }

  state(alignBehindBall)
  {
    transition
    {
      if(common.between(ball.positionRobot.y(), -10.f, 10.f)
          && common.between(ball.positionRobot.x(), kickX -10.f, kickX + 20.f)
          && fabs(goals.angleToGoal) < common.fromDegrees(3.f))
        goto kick;
      if (ball.distance > ballDisThreshold || fabs(ball.angleDeg) > 90)
        goto walkToBall;
    }
    action
    {
      //theHeadControlMode = HeadControl::lookAtBall;
      theHeadControlMode = (state_time%4000<2000) ?
          HeadControl::lookAtBall : HeadControl::lookLeftAndRight;
      WalkToTarget(Pose2f(0.60f, 0.60f, 0.60f),
                   Pose2f(goals.angleToGoal, ball.positionRobot.x() - kickX, ball.positionRobot.y()));
    }
  }

  state(kickLeft)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      InWalkKick(WalkRequest::left,
                 Pose2f(goals.angleToGoal, ball.positionRobot.x() - kickX, ball.positionRobot.y() - 50.f));
    }
  }

  state(kickRight )
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      InWalkKick(WalkRequest::right,
                 Pose2f(goals.angleToGoal, ball.positionRobot.x() - kickX, ball.positionRobot.y() + 50.f));
    }
  }

}

