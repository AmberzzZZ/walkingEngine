option (LookAtBall)
{
  initial_state (lookAtBall)
  {
    action
    {
      float ballGlobalRelX = (Geometry::fieldCoord2Relative(theRobotPose,Vector2f(ball.teamPosition.x(),ball.teamPosition.y()))).x();
      float ballGlobalRelY = (Geometry::fieldCoord2Relative(theRobotPose,Vector2f(ball.teamPosition.x(),ball.teamPosition.y()))).y();
      SetHeadTargetOnGround (Vector3f (ballGlobalRelX, ballGlobalRelY, 50.f));
    }
  }

}
