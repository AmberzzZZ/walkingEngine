/*
 *@Usage:Ball-searching method used by striker,supporter and stabber
 */

option(SearchForBallTogether, (float)(9.f) searchPoseAngleDegError,
       (float)(.7f) speed, (float)(100.f) errSearchPose)
{
  const Pose2f keeperSearchPose(0.f, -4100.f, 0.f);
  const Pose2f strikerSearchPose(0.f,1000.f,-1000.f);
  const Pose2f supporterSearchPose(0.f,-1500.f,1500.f);
  const Pose2f defenderSearchPose(0.f,-2500.f,-1300.f);
  const Pose2f stabberSearchPose(0.f,3000.f,1500.f);
  Pose2f searchPose;

//  switch(theRobotInfo.number)
//  {
//  case 1:
//    searchPose = keeperSearchPose;
//  case 2:
//    searchPose = strikerSearchPose;
//  case 3:
//    searchPose = supporterSearchPose;
//  case 4:
//    searchPose = defenderSearchPose;
//  case 5:
//    searchPose = stabberSearchPose;
//  }

  if(theRole.role == 1)
    searchPose = keeperSearchPose;
  else if(theRole.role == 2)
    searchPose = strikerSearchPose;
  else if(theRole.role == 3)
    searchPose = supporterSearchPose;
  else if(theRole.role == 4)
    searchPose = defenderSearchPose;
  else if(theRole.role == 5)
    searchPose = stabberSearchPose;
  else
    searchPose = strikerSearchPose;


  initial_state(walkToSearchPose)
  {
    transition
    {
      if(common.between(robot.x,searchPose.translation.x()
                        - errSearchPose,searchPose.translation.x()
                        + errSearchPose) && common.between(robot.y,searchPose.translation.y()
                                - errSearchPose,searchPose.translation.y()
                                + errSearchPose))//TODO??
      {
        goto turnToSearchBall;
      }
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRight;
      WalkToPose(searchPose);
    }
  }

  state(turnToSearchBall)
  {
    transition
    {
      if(action_done)
        goto turnToSearchBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookLeftAndRightFast;
      WalkAtSpeedPercentage(Pose2f(0.7f, 0.f, 0.f));
    }
  }
}
