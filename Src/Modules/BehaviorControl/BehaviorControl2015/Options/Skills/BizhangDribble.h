option(BizhangDribble, (float) x, (float) y)
{
  std::cout<<"**************************"<<std::endl;
  float obstacle_x[9];
  float obstacle_y[9];
  Vector2f obstacle_global;
  float obstacle_global_x[9];
  float obstacle_global_y[9];
  int i=0;
  float a,b,distance=0;
  int j=0;
  int k=0;
  for (const Obstacle & obstacle : theObstacleModel.obstacles)
  {

	  if(obstacle.center.x()>50 && (obstacle.center.x()*cos(robot.rotation)-obstacle.center.y()*sin(robot.rotation)+robot.x)<4500)
	  {
		  std::cout<<i<<std::endl;

	 obstacle_x[i]=obstacle.center.x();
	 obstacle_y[i]=obstacle.center.y();
//	 obstacle_global=Transformation::robotToField(robot.pose,obstacle.center);
//	 obstacle_x[i]=obstacle_global.x();
//	 obstacle_y[i]=obstacle_global.y();
	 obstacle_global=Transformation::robotToField(theRobotPose,obstacle.center);
//	 obstacle_global_x[i]=obstacle.center.x()*cos(robot.rotation)-obstacle.center.y()*sin(robot.rotation)+robot.x;
//	 obstacle_global_y[i]=obstacle.center.y()*cos(robot.rotation)+obstacle.center.x()*sin(robot.rotation)+robot.y;
	 obstacle_global_x[i]=obstacle_global.x();
	 obstacle_global_y[i]=obstacle_global.y();
	 std::cout<<"obstacle.x-->: "<<obstacle_x[i]<<std::endl;
	 		  std::cout<<"obstacle.y-->: "<<obstacle_y[i]<<std::endl;
	 		  std::cout<<"obstacle.global.x-->: "<<obstacle_global_x[i]<<std::endl;
	 		  std::cout<<"obstacle.global.y-->: "<<obstacle_global_y[i]<<std::endl;
	  }


	 i++;
	 std::cout<<"---------------------------"<<std::endl;
  }
  a=(y-ball.positionField.y())/(x-ball.positionField.x());
  b=y-a*x;
  for(j=0;j<=i;j++)
  {
	  distance=fabs(a*(obstacle_global_x[j])+b-obstacle_global_y[j])/(a*a+1);
	  std::cout<<distance<<std::endl;
	  Vector2f p(obstacle_global_x[j],obstacle_global_y[j]);
	  std::cout<<p<<std::endl;
	  if(distance<320 && obstacle_x[j]>0 && ((p-ball.global).norm())<700)
		  {
		  k++;
		  break;
		  }
	  std::cout<<k<<std::endl;
  }

  common_transition
  {

  }

	initial_state(start)
{
	transition
	{
//		if (state_time > 0)
			goto Dribble;
	}
	action
	{
//		Stand();
	}
}

state(ChangeTrack)
{
	float x_new,y_new;
	transition
	{
		//if(k==0 && fabs(ball.global.x())>fabs(x_new))
			if(distance>320)
			goto Dribble;
	}
	action
	{
	  x_new=obstacle_global_x[j];
	  if(fabs(x_new)<3000 || fabs(ball.global.y())<1100)
	  {
	  if((a*(obstacle_global_x[j])+b)>obstacle_global_y[j])
		  y_new=obstacle_global_y[j]+500;
	  else
		  y_new=obstacle_global_y[j]-500;
	  }
	  else
	  {
		  if(ball.global.y()>1100)
		  		  y_new=obstacle_global_y[j]-500;
		  else
		  		  y_new=obstacle_global_y[j]+500;
	  }

	  Dribble(x_new,y_new);
	}
}
state(Dribble)
{
	transition
	{
		if(k!=0)
			goto ChangeTrack;
		if (fabs(ball.global.x() - x) < 150
				&& fabs(ball.global.y() - y) < 150 && ball.distance < 200)
			goto finish;

	}
	action
	{
		Dribble(x,y);
	}
}


target_state(finish)
{
  transition
  {
  }
  action
  {
    theHeadControlMode=HeadControl::lookLeftAndRight;
//    Stand();
  }
}


}
