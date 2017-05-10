//@SiQiang
//option(WalkToPose_SiQiang,(Vector2f)(Vector2f(4500.f, 0.f)) target)
option(WalkToPose_SiQiang, (Pose2f) pose, (Pose2f)(Pose2f(5_deg, 100.f, 100.f)) delta, (bool)(false) pause)
{
	static Pose2f prePose = Pose2f((Angle)179_deg,6000.f,9000.f);
	static Vector2f target,assignTarget;//global
	//robot to global
	//if pose changes, we update the target on field, and use the target to update the targetToRobot(because theRobotPose is changing, so the targetRobot is also changing)
	if(pose != prePose)
	{
		target = pose.translation;
		float distance = 2000.f;//the distance from target to assigntarget
		assignTarget = target + Vector2f(distance*cos(pose.rotation),distance*sin(pose.rotation));
		prePose = pose;
//		target = Transformation::robotToField(theRobotPose,target);//target from robot to field
//		assignTarget = Transformation::robotToField(theRobotPose,assignTarget);
	}

	float kx,ky,kthetaGamma,kthetaPhi;
	walkToTarget.GetAllParameterValue(kx,ky,kthetaGamma,kthetaPhi,target,assignTarget);
//	std::cout<<"walkToTarget.gammaValue---------->"<<walkToTarget.gammaValue<<std::endl;
//	std::cout<<"walkToTarget.rhoValue---->"<<walkToTarget.rhoValue<<std::endl;
//	std::cout<<"walkToTarget.gammaValue---------->"<<walkToTarget.gammaValue<<std::endl;

	Vector2f targetToRobot;
	targetToRobot = theRobotPose.inverse() * target;

  initial_state(main)
  {
	  transition{
		if(fabs(walkToTarget.gammaValue) > (Angle)60_deg)
			goto turnAround;
		else
			goto alignToTarget;

	  }
    action
    {
    	Stand();
    }
  }

  state(turnAround)
  {
	  transition{
		if(fabs(walkToTarget.gammaValue) <= (Angle)60_deg)
			goto alignToTarget;
	  }
	  action{
		  if(walkToTarget.gammaValue > (Angle)0_deg)
			  WalkAtSpeedPercentage(Pose2f(0.7f,0.f,0.f));
		  else
			  WalkAtSpeedPercentage(Pose2f(-0.7f,0.f,0.f));
	  }
  }

  state(alignToTarget){
	  transition{
			if(fabs(walkToTarget.gammaValue) > (Angle)60_deg)
				goto turnAround;
	  }
	  action{
//		  Stand();
		  float vx,vy,vt;
		  if(fabs(walkToTarget.gammaValue) > (Angle)15_deg)
		  {
			  kx = 0.01f;
		  }
		  else if(walkToTarget.rhoValue > 300.f)
		  {
			  kx = 1.f;
		  }
		  if(walkToTarget.rhoValue > 300.f)
		  {
			  ky = ky/4.f;
		  }
		  if(walkToTarget.rhoValue > 350.f || walkToTarget.rhoValue < 150.f)
		  {
			  kthetaPhi = kthetaPhi/3.f;
		  }
		  vx = kx*walkToTarget.rhoValue/800.f;
		  vy = ky*walkToTarget.phiValue/0.5f;
		  vt = kthetaGamma*walkToTarget.gammaValue*(walkToTarget.rhoValue/(walkToTarget.rhoValue+1.f))*2.f-kthetaPhi*walkToTarget.phiValue*(1.f/(walkToTarget.rhoValue+1.f))*2.f;
//		  if(fabs(walkToTarget.gammaValue)<(Angle)15_deg && fabs(walkToTarget.phiValue)<(Angle)15_deg)
//		  {
//			  vx = 0.7f;
//		  }
		  if(vx<-0.7f) vx = -0.7f;if(vx > 0.7f) vx = 0.7f;
		  if(vy<-0.7f) vy = -0.7f;if(vy > 0.7f) vy = 0.7f;
		  if(vt<-0.7f) vt = -0.7f;if(vt > 0.7f) vt = 0.7f;

		  WalkAtSpeedPercentage(Pose2f(vt,vx,vy));
	  }
  }
}
