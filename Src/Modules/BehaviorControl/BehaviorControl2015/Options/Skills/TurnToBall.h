/**
 * @file TurnToBall.h
 * @author Hanbin Wang
 * @date Mar 18, 2014
 * @Usage TurnToBall(); or TurnToBall(10.f, 0.8f)
 * @param ballAngleDegError the error of direction to ball relative to robot
 *                          that you can torlerance in degrees(0 is front,90 is left)
 */

option(TurnToBall, (float)(9.f) ballAngleDegError, (float)(.7f) speed)
{
  initial_state(turn)
  {
    transition
    {
      if(fabs(ball.angleDeg) < ballAngleDegError)
        goto finished;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2f(speed, 0, 0),
                   Pose2f(ball.angleRad, 0.f, 0.f));
    }
  }

  target_state(finished)
  {
    transition
    {
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
//      WalkToTarget(Pose2f(speed, speed, speed),
//                   Pose2f(ball.angleRad, 0.f, 0.f));
    }
  }
}
