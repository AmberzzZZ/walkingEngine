/**
 * @TODO Transplant this file from tjark2015
 * @author Zhongde Chen
 * @Date March 2nd
 * */
option(LookAtTeamBall)
{
  initial_state(lookAtTeamBall)
  {
    action
    {
      float ballGlobalRelX = (Geometry::fieldCoord2Relative(theRobotPose,Vector2f(ball.teamPosition.x(),ball.teamPosition.y()))).x();
      float ballGlobalRelY = (Geometry::fieldCoord2Relative(theRobotPose,Vector2f(ball.teamPosition.x(),ball.teamPosition.y()))).y();
      SetHeadTargetOnGround(Vector3f(ballGlobalRelX,ballGlobalRelY,35.f));
    }
  }

}
