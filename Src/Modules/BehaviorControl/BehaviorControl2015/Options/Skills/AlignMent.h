option(AlignMent,(Vector2f)(Vector2f(4500.f, 0.f)) target)
{
	#define speedMax 0.7
	float kx,ky,kthetaGamma,kthetaPhi;
	static bool useLeftOrRight = true;//true--left,false--right
	static int k = 1;//just decide vx
	Angle angleToTarget = (theRobotPose.inverse() * target).angle();

	alignFuzzy.GetAllParameterValue(kx,ky,kthetaGamma,kthetaPhi,target,useLeftOrRight);
	 if(alignFuzzy.rhoValue > 200.f && alignFuzzy.phiValue >= 0.f)
	 {
		useLeftOrRight = true;
	 }
	 else if(alignFuzzy.rhoValue > 200.f && alignFuzzy.phiValue < 0.f)
	 {
		useLeftOrRight = false;
	 }
  initial_state(main)
  {
	  transition{
//		if(!ball.isTeamPositionValid && ball.notSeenTime()>2000.f)
//			goto searchForball;
//				else if(alignFuzzy.rhoValue > 2000.f)
//					goto walkTotarget;
//		else
			goto turnAround;


	  }
    action
    {
    	Stand();
    }
  }
  state(searchForball)
  {
	  transition{
		if(ball.wasSeen())
		    goto turnAround;
	  }
	  action{
		  SearchForBall();
	  }
  }
  state(turnAround)
  {
	  transition{
		if(ball.wasSeen() && fabs(alignFuzzy.gammaValue) <= (Angle)60_deg)
			goto alignToBall;
	  }
	  action{
		  if(alignFuzzy.gammaValue > (Angle)0_deg)
			  WalkAtSpeedPercentage(Pose2f(speedMax,0.f,0.f));
		  else
			  WalkAtSpeedPercentage(Pose2f(-speedMax,0.f,0.f));
	  }
  }

  state(alignToBall){
	  transition{
//		if(alignFuzzy.rhoValue < 200.f)
//			goto stand;
//		if(fabs(alignFuzzy.gammaValue) > (Angle)90_deg)
//			goto turnAround;
	  }
	  action{
//		  Stand();
		  float vx,vy,vt;
		  if(fabs(alignFuzzy.gammaValue) > (Angle)15_deg && alignFuzzy.rhoValue > 400.f)
		  {
			  kx = 0.1f;
		  }
		  else if(alignFuzzy.rhoValue > 200.f)
		  {
			  kx = 1.f;
		  }
		  if(alignFuzzy.rhoValue > 350.f)
		  {
			  ky = ky/2.f;
		  }
		  else if(alignFuzzy.rhoValue > 200)
		  {
		  	  ky = ky *2.f;
		  }
		  if(alignFuzzy.rhoValue > 150.f)// || alignFuzzy.rhoValue < 50.f)
		  {
			  kthetaPhi = kthetaPhi/2.f;
		  }
		  vx = kx*alignFuzzy.rhoValue/600.f;
		  vy = ky*alignFuzzy.phiValue/0.5f;
		  vt = kthetaGamma*alignFuzzy.gammaValue*(alignFuzzy.rhoValue/(alignFuzzy.rhoValue+1.f))*3.f-kthetaPhi*alignFuzzy.phiValue*(1.f/(alignFuzzy.rhoValue+1.f))*3.f;
//		  
		  if(k > 0 && alignFuzzy.rhoValue < 90.f)
		  {
		  	k = -1;
		  	// vt = kthetaPhi*(theRobotPose.inverse() * target).angle()*3.f;
		  }
		  else if(k < 0 && alignFuzzy.rhoValue >180.f)
		  {
		  	k = 1;
		  }
		  if(alignFuzzy.rhoValue < 200.f ) 
		  {
		  	vt = kthetaPhi*angleToTarget*3.f;
		  	if(common.between(angleToTarget, -5_deg, 5_deg) && fabs(alignFuzzy.phiValue) < (Angle)20_deg)
		  	{
		  		vy = fabs(vy);
	  			if(useLeftOrRight)
	  			{
	  				if(ball.positionRobot.y() > 60.f)
	  					vy = vy *3.f;
	  				else if(ball.positionRobot.y() < 20.f)
	  					vy = -vy *3.f;
	  				else
	  					vy = vy/4.f;
	  			}
	  			else
	  			{
	  				if(ball.positionRobot.y() < -60.f)
	  					vy = -vy *3.f;
	  				else if(ball.positionRobot.y() > -20.f)
	  					vy = vy *3.f;
	  				else
	  					vy = vy/4.f;
	  			}

		  	}
		  }
		  vx = k*vx;
		  if(vx < speedMax/2.f && vx > 0.f && alignFuzzy.rhoValue > 200) vx = speedMax/2.f;
		  if(vx<-speedMax) vx = -speedMax;if(vx > speedMax) vx = speedMax;
		  if(vy<-speedMax) vy = -speedMax;if(vy > speedMax) vy = speedMax;
		  if(vt<-speedMax) vt = -speedMax;if(vt > speedMax) vt = speedMax;

		  //limit vx,vy,vt
		  if(fabs(vx) > speedMax/2.f && fabs(vy) > speedMax/2.f && fabs(vt) > speedMax/2.f) 
		  {
		  	vx = vx/3.f;
		  	vy = vy/2.f;
		  }
		  else if(fabs(vx) > speedMax/2.f && fabs(vy) > speedMax/2.f)
		  {
		  	vy = vy/2.f;
		  	vx = vx;
		  }
		  // std::cout<<"useLeftOrRight---->"<<useLeftOrRight<<std::endl;
		  // std::cout<<"rho--------------->"<<alignFuzzy.rhoValue<<std::endl;
		  // std::cout<<"judge------------->"<<fabs(ball.global.y() - robot.y) - 70.f<<std::endl;
		  // std::cout<<"phi--------------->"<<alignFuzzy.phiValue<<std::endl;
		  // std::cout<<"vy----------------->"<<vy<<std::endl;
		  WalkAtSpeedPercentage(Pose2f(vt,vx,vy));

		  // Vector2f BallToTarget = target - ball.global;
		  // float backwards = 200.f;
		  // float sidewards = 50.f;
		  // Vector2f backwardVector = Vector2f(backwards*cos(BallToTarget.angle()),backwards*sin(BallToTarget.angle()));
		  // Vector2f sidewardsVector;
		  // if(useLeftOrRight)
		  // 	sidewardsVector = Vector2f(sidewards*cos(BallToTarget.angle()-90_deg),sidewards*sin(BallToTarget.angle()-90_deg));
		  // else
		  // 	sidewardsVector = Vector2f(sidewards*cos(BallToTarget.angle()+90_deg),sidewards*sin(BallToTarget.angle()+90_deg));		  	
		  // WalkToPose(Pose2f(BallToTarget.angle(), (ball.global-backwardVector+sidewardsVector).x(), (ball.global-backwardVector+sidewardsVector).y() ), Pose2f(5_deg, 20.f, 20.f));
  
	  }
  }
  state(stand){
	  transition{

	  }
	  action{
		  Stand();
	  }
  }
}
