option(KickBallToPose, (Vector2f)(Vector2f(4500.f, 0.f)) kickPose)
{
//  const bool shootLeft = false;
//  const bool shootRight = false;

//  const bool shootBackLeft = false;
//  const bool shootBackRight = false;

  const float kickX = 190.f;
  const float kickY = 45.f;
  const float kickMargin_x = 10.f;
  const float kickMargin_y = 10.f;

  const bool inWalkKickLeft = false;
  const float inWalkKickLeft_x = 140.f;
  const float inWalkKickLeft_y = -10.f;

  const bool inWalkKickRight = false;
  const float inWalkKickRight_x = 140.f;
  const float inWalkKickRight_y = 10.f;

  const bool kickOutsideLeft = false;
  const float kickOutsideLeft_x = 190.f;
  const float kickOutsideLeft_y = 140.f;

  const bool kickOutsideRight = false;
  const float kickOutsideRight_x = 190.f;
  const float kickOutsideRight_y = -140.f;

  const bool kickInsideLeft = false;
  const float kickInsideLeft_x = 190.f;
  const float kickInsideLeft_y = -8.f;

  const bool kickInsideRight = false;
  const float kickInsideRight_x = 190.f;
  const float kickInsideRight_y = 8.f;

  Pose2f p(theRobotPose.rotation, ball.global);
  Vector2f v = p.inverse() * kickPose;
  float angleToShoot = v.angle();
  float distanceToShoot = v.norm();

  common_transition
  {

  }

  initial_state(turnToBall)
  {
    transition
    {
      if(fabs(ball.angleRad) < 5_deg)
        goto walkToBall;
    }
    action
    {
      TurnToBall();
    }
  }

  state(walkToBall)
  {
     transition
     {
       if(fabs(ball.angleRad) >= 10_deg)
         goto turnToBall;
       else if(ball.distance < 500.f)
         goto checkKickFlag;
     }
     action
     {
       WalkToBall();
     }
  }

  state(checkKickFlag)
  {
//    const bool shootBack = shootBackLeft || shootBackRight;
    const bool kickLeft45 = kickOutsideLeft || kickInsideLeft;
    const bool kickRight45 = kickOutsideRight || kickInsideRight;
    transition
    {
      if(distanceToShoot > 4500.f)
      {
//        std::cout<<"distance--> "<<distanceToShoot<<std::endl;
        goto kickForward;
      }
      else if(fabs(v.angle()) < 30_deg)
      {
//        std::cout<<"angle--> "<<v.angle()<<std::endl;
        goto kickForward;
      }
      else if(v.angle() > 30_deg && v.angle() < 60_deg && kickLeft45)
        goto kickLeft45;
      else if(v.angle() > -60_deg && v.angle() < -30_deg && kickRight45)
        goto kickRight45;
      else if(v.angle() > 60_deg && v.angle() < 120_deg && inWalkKickLeft)
        goto inWalkKickLeft;
      else if(v.angle() > -120_deg && v.angle() < -60_deg && inWalkKickRight)
        goto inWalkKickRight;
      else
      {
//        std::cout<<"distance--> "<<distanceToShoot<<std::endl;
//        std::cout<<"angle--> "<<v.angle()<<std::endl;
        goto kickForward;
      }
    }
    action
    {
      WalkToBall();
    }
  }

  state(kickForward)
  {
    float poseDistance = (kickPose - robot.pose.translation).norm();
    transition
    {
      if(ball.distance > 1000.f)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      if(common.between(theBallModel.estimate.position.y(), -kickY - kickMargin_y, -kickY + kickMargin_y)
          && common.between(theBallModel.estimate.position.x(), kickX - kickMargin_x, kickX + kickMargin_x)
          && fabs(angleToShoot) < 5_deg)
      {
        KickForward(KickRequest::kickForward,false, poseDistance);
//        goto turnToBall;
      }
      else if(common.between(theBallModel.estimate.position.y(), kickY - kickMargin_y, kickY + kickMargin_y)
              && common.between(theBallModel.estimate.position.x(), kickX - kickMargin_x, kickX + kickMargin_x)
              && fabs(angleToShoot) < 5_deg)
      {
        KickForward(KickRequest::kickForward,true, poseDistance);
//        goto turnToBall;
      }
      else
      {
        if(0 && ball.positionRobot.y() >= 0)
        {
          WalkToTarget(Pose2f(0.30f, 0.50f, 0.50f),
                     Pose2f(angleToShoot, ball.positionRobot.x() - kickX, ball.positionRobot.y() - kickY));
        }
        else
        {
          WalkToTarget(Pose2f(0.30f, 0.50f, 0.50f),
                     Pose2f(angleToShoot, ball.positionRobot.x() - kickX, ball.positionRobot.y() + kickY));
        }
      }
    }
  }

  state(kickLeft45)
  {
    transition
    {
      if(ball.distance > 1000.f)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      if(common.between(theBallModel.estimate.position.y(), kickInsideLeft_y - kickMargin_y, kickInsideLeft_y + kickMargin_y)
          && common.between(theBallModel.estimate.position.x(), kickInsideLeft_x - kickMargin_x, kickInsideLeft_x + kickMargin_x)
          && fabs(angleToShoot - 45_deg) < 5_deg)
      {
        KickForward(KickRequest::kickInsideLeft,false);
//        goto turnToBall;
      }
      else if(common.between(theBallModel.estimate.position.y(), kickOutsideLeft_y - kickMargin_y, kickOutsideLeft_y + kickMargin_y)
              && common.between(theBallModel.estimate.position.x(), kickOutsideLeft_x - kickMargin_x, kickOutsideLeft_x + kickMargin_x)
              && fabs(angleToShoot - 45_deg) < 5_deg)
      {
        KickForward(KickRequest::kickOutsideRight,true);//kickOutsideLeft
//        goto turnToBall;
      }
      else
      {
        if(!kickOutsideLeft)
        {
          WalkToTarget(Pose2f(0.30f, 0.50f, 0.50f),
                     Pose2f(angleToShoot - 45_deg, ball.positionRobot.x() - kickInsideLeft_x, ball.positionRobot.y() - kickInsideLeft_y));
        }
        else if(!kickInsideLeft)
        {
          WalkToTarget(Pose2f(0.30f, 0.50f, 0.50f),
                     Pose2f(angleToShoot - 45_deg, ball.positionRobot.x() - kickOutsideLeft_x, ball.positionRobot.y() - kickOutsideLeft_y));
        }
        else if(ball.y <= 50)
        {
          WalkToTarget(Pose2f(0.30f, 0.50f, 0.50f),
                     Pose2f(angleToShoot - 45_deg, ball.positionRobot.x() - kickInsideLeft_x, ball.positionRobot.y() - kickInsideLeft_y));
        }
        else
        {
          WalkToTarget(Pose2f(0.30f, 0.50f, 0.50f),
                     Pose2f(angleToShoot - 45_deg, ball.positionRobot.x() - kickOutsideLeft_x, ball.positionRobot.y() - kickOutsideLeft_y));
        }
      }
    }
  }

  state(kickRight45)
  {
    transition
    {
      if(ball.distance > 1000.f)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      if(common.between(theBallModel.estimate.position.y(), kickInsideRight_y - kickMargin_y, kickInsideRight_y + kickMargin_y)
          && common.between(theBallModel.estimate.position.x(), kickInsideRight_x - kickMargin_x, kickInsideRight_x + kickMargin_x)
          && fabs(angleToShoot + 45_deg) < 5_deg)
      {
        KickForward(KickRequest::kickInsideLeft,true); //kickInsideRight
//        goto turnToBall;
      }
      else if(common.between(theBallModel.estimate.position.y(), kickOutsideRight_y - kickMargin_y, kickOutsideRight_y + kickMargin_y)
              && common.between(theBallModel.estimate.position.x(), kickOutsideRight_x - kickMargin_x, kickOutsideRight_x + kickMargin_x)
              && fabs(angleToShoot + 45_deg) < 5_deg)
      {
        KickForward(KickRequest::kickOutsideRight);//kickOutsideRight
//        goto turnToBall;
      }
      else
      {
        if(!kickOutsideRight)
        {
          WalkToTarget(Pose2f(0.30f, 0.50f, 0.50f),
                     Pose2f(angleToShoot + 45_deg, ball.positionRobot.x() - kickInsideRight_x, ball.positionRobot.y() - kickInsideRight_x));
        }
        else if(!kickInsideRight)
        {
          WalkToTarget(Pose2f(0.30f, 0.50f, 0.50f),
                     Pose2f(angleToShoot + 45_deg, ball.positionRobot.x() - kickOutsideRight_x, ball.positionRobot.y() - kickOutsideRight_y));
        }
        else if(ball.y >= -50)
        {
          WalkToTarget(Pose2f(0.30f, 0.50f, 0.50f),
                     Pose2f(angleToShoot + 45_deg, ball.positionRobot.x() - kickInsideRight_x, ball.positionRobot.y() - kickInsideRight_y));
        }
        else
        {
          WalkToTarget(Pose2f(0.30f, 0.50f, 0.50f),
                     Pose2f(angleToShoot + 45_deg, ball.positionRobot.x() - kickOutsideRight_x, ball.positionRobot.y() - kickOutsideRight_y));
        }
      }
    }
  }

  state(inWalkKickLeft)
  {
    transition
    {
      if(ball.distance > 1000.f)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      if(common.between(theBallModel.estimate.position.y(), inWalkKickLeft_y - kickMargin_y, inWalkKickLeft_y + kickMargin_y)
          && common.between(theBallModel.estimate.position.x(), inWalkKickLeft_x - kickMargin_x, inWalkKickLeft_x + kickMargin_x)
          && fabs(angleToShoot - 90_deg) < 5_deg)
      {
        InWalkKick(WalkRequest::sidewardsRight,
                   Pose2f(angleToShoot - 90_deg, ball.positionRobot.x() - inWalkKickLeft_x, ball.positionRobot.y() - inWalkKickLeft_y));
//        goto turnToBall;
      }
      else
      {
        WalkToTarget(Pose2f(0.30f, 0.50f, 0.50f),
                Pose2f(angleToShoot - 90_deg, ball.positionRobot.x() - inWalkKickLeft_x, ball.positionRobot.y() - inWalkKickLeft_y));
      }
    }
  }

  state(inWalkKickRight)
  {
    transition
    {
      if(ball.distance > 1000.f)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      if(common.between(theBallModel.estimate.position.y(), inWalkKickRight_y - kickMargin_y, inWalkKickRight_y + kickMargin_y)
          && common.between(theBallModel.estimate.position.x(), inWalkKickRight_x - kickMargin_x, inWalkKickRight_x + kickMargin_x)
          && fabs(angleToShoot + 90_deg) < 5_deg)
      {
        InWalkKick(WalkRequest::sidewardsLeft,
                   Pose2f(angleToShoot + 90_deg, ball.positionRobot.x() - inWalkKickRight_x, ball.positionRobot.y() - inWalkKickRight_y));
//        goto turnToBall;
      }
      else
      {
        WalkToTarget(Pose2f(0.30f, 0.50f, 0.50f),
                Pose2f(angleToShoot + 90_deg, ball.positionRobot.x() - inWalkKickRight_x, ball.positionRobot.y() - inWalkKickRight_y));
      }
    }
  }

}
