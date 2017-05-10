option (LookAtOppKeeper)
{
  initial_state (lookAtOppKeeper)
  {
    action
    {
      SetHeadTargetOnGround (Vector3f (obstacle.oppKeeper.x(), obstacle.oppKeeper.y(), 300.f));
    }
  }
}
