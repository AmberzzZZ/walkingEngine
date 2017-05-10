option(RoleAssign)
{
	common_transition
	{

	}

  initial_state(assignTheRole)
  {
    transition
    {   
    }
    action
    {
      if(theRole.role == Role::striker)
      {
        Striker();
      }
      else if(theRole.role == Role::keeper)
      {
        Keeper();
      }
      else if(theRole.role == Role::supporter)
      {
        Supporter2();
      }
      else if(theRole.role == Role::defender)
      {
        Defender();
      }
      else
      {
        Stabber();
      }
    }
  }
}
