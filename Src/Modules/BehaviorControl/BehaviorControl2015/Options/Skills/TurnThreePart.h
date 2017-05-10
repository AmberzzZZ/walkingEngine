option(TurnThreePart, (bool)(false) isMirror, (Pose2f)(Pose2f(0.7f, 0.f, 0.f)) speed)
{
//    Pose2f speed(.7f, 0.f, 0.f);
#ifdef TARGET_ROBOT
    float angleCorrected = 20_deg;
#else
    float angleCorrected = 40_deg;
#endif
    if(isMirror) {
        speed = Pose2f(-0.7f,0.f,0.f);
    }
    


 Vector2f refPoint = Vector2f(4500.f,0.f);
 Vector2f refPointRelative;
   refPointRelative = theRobotPose.inverse() * refPoint;
 Angle refAngle;
     refAngle = refPointRelative.angle(); //机器人坐标系 球门的角度
 static int firstStep_flag = 1;
 static int inital_flag = 1;
 static int count = 0;

 initial_state(start)
 {
	 transition
	 {

        if(isMirror)
	    {
        	if(refAngle > 60_deg && fabs(refAngle) < 120_deg)
		    {
        		firstStep_flag = 5;
//		        inital_flag = 0;
		        goto fifth;
     	    }
		     else if(fabs(refAngle) > 120_deg)
		    {
		        firstStep_flag = 3;
//		        inital_flag = 0;
		        goto third;
		    }
		     else
		    {
		     firstStep_flag = 1;
		     goto first;
		    }
		 }
		 else
         {
		     if(refAngle < -60_deg && fabs(refAngle) < 120_deg)
		     {
		        firstStep_flag = 3;
//		        inital_flag = 0;
		        goto third;
		      }
             else if(fabs(refAngle) > 120_deg)
		      {
		        firstStep_flag = 5;
//		        inital_flag = 0;
		        goto fifth;
		      }
		     else
		     {
		       firstStep_flag = 1;
		       goto first;
		     }
		  }
	 }
	 action
	 {
		 theHeadControlMode = HeadControl::lookLeftAndRightFast;
		  Stand();
	 }
 }

  state(first)
  {
      transition
      {
        if(state_time > 1000 && theWalkingEngineOutput.actionFlag == 0 || fabs(refAngle) < 5_deg)
        {
            // if(inital_flag == 0)
            //    goto finish;
            // else
            // {
            //   firstStep_flag = 1;
            //   goto second;
            // }
          if(isMirror)
          {
            if(firstStep_flag == 1)
              goto second;
            else if(firstStep_flag == 3)
              goto fourth;
            else
              goto sixth;
          }
          else
          {
            if(firstStep_flag == 1)
              goto second;
            else if(firstStep_flag == 3)
              goto sixth;
            else
              goto fourth;
          }
        }
      }


      action
      {
          // WalkToTarget(speed,Pose2f(90_deg + angleCorrected,0.f,0.f));  //@YY
   	        theHeadControlMode = state_time % 6000 < 3000? HeadControl::lookDown : HeadControl::lookForward;
   	        WalkToTarget(speed,Pose2f(refAngle,0.f,0.f));
      }
  }

  state(second)
  {
      transition
      {
        
        // if(state_time > 1000 && theWalkingEngineOutput.actionFlag == 0)
        // if(theHeadJointRequest.pan == 0_deg)
        //   count++;
        // if(state_time > 3000)
        //   count++;
        if(state_time > 8000)
        {
          if(isMirror)
            {
               if(firstStep_flag == 5)
                  goto third;
               else if(firstStep_flag == 3)
                  goto first;
                else
                  goto fifth; 
            }
            else
            {
               if(firstStep_flag == 3)
                  goto fifth;
                else if(firstStep_flag == 5)
                  goto first;
                else
                  goto third;
            }
        }

        // if(count == 1)
        // {
            
        //     if(isMirror)
        //     {
        //        if(firstStep_flag == 4)
        //           goto third;
        //        else if(firstStep_flag == 3)
        //           goto first;
        //         else
        //           goto fifth; 
        //     }
        //     else
        //     {
        //        if(firstStep_flag == 3)
        //           goto fifth;
        //         else if(firstStep_flag == 4)
        //           goto first;
        //         else
        //           goto third;
        //     }
            
        // }
        // if(count == 3)
        // {
            
        //     if(isMirror)
        //     {
        //        if(firstStep_flag == 4)
        //           goto first;
        //        else if(firstStep_flag == 3)
        //           goto fifth;
        //         else
        //           goto third; 
        //     }
        //     else
        //     {
        //        if(firstStep_flag == 3)
        //           goto first;
        //         else if(firstStep_flag == 4)
        //           goto third;
        //         else
        //           goto fifth;
        //     }
        // }
        // if(count == 5)
        // {
          
        //     if(isMirror)
        //     {
        //        if(firstStep_flag == 4)
        //           goto fifth;
        //        else if(firstStep_flag == 3)
        //           goto third;
        //        else
        //           goto first; 
        //     }
        //     else
        //     {
        //        if(firstStep_flag == 3)
        //           goto third;
        //         else if(firstStep_flag == 4)
        //           goto fifth;
        //         else
        //           goto first;
        //     }
        // }
        // if(count > 5)
        // {

        //     count = 0;
        //     goto finish;
        // }
      }
      action
      {
         theHeadControlMode = HeadControl::lookLeftAndRightFast;
         Stand();
      }
  }

  state(third)
  {
      transition
      {
        if(state_time > 1000 && theWalkingEngineOutput.actionFlag == 0 && fabs(refAngle) >= 120_deg)
        {
            // count++;
            // goto second;
          if(isMirror)
          {
            if(firstStep_flag == 1)
              goto sixth;
            else if(firstStep_flag == 3)
              goto second;
            else
              goto fourth;
          }
          else
          {
            if(firstStep_flag == 1)
              goto fourth;
            else if(firstStep_flag == 3)
              goto second;
            else
              goto sixth;
          }
        }
      }
      action
      {
          theHeadControlMode = state_time % 6000 < 3000? HeadControl::lookDown : HeadControl::lookForward;
//          WalkToTarget(speed,Pose2f(refAngle + 120_deg,0.f,0.f));
          WalkAtSpeedPercentage(Pose2f(0.7, 0.f, 0.f));
      }
  }

  state(fourth)
  {
      transition
      {
          if(state_time > 8000)
          {
              if(isMirror)
            {
               if(firstStep_flag == 5)
                  goto first;
               else if(firstStep_flag == 3)
                  goto fifth;
                else
                  goto third; 
            }
            else
            {
               if(firstStep_flag == 3)
                  goto first;
                else if(firstStep_flag == 5)
                  goto third;
                else
                  goto fifth;
            }
          }
      }

      action
      {
          theHeadControlMode = HeadControl::lookLeftAndRightFast;
          Stand();
      }
  }

  state(fifth)
  {
      transition
      {
        if(state_time > 1000 && theWalkingEngineOutput.actionFlag == 0 && fabs(refAngle) <= 120_deg)
        {
            // count++;
            // goto second;
          if(isMirror)
          {
            if(firstStep_flag == 1)
              goto fourth;
            else if(firstStep_flag == 3)
              goto sixth;
            else
              goto second;
          }
          else
          {
            if(firstStep_flag == 1)
              goto sixth;
            else if(firstStep_flag == 3)
              goto fourth;
            else
              goto second;
          }
        }
      }
      action
      {
          theHeadControlMode = state_time % 6000 < 3000? HeadControl::lookDown : HeadControl::lookForward;
//          WalkToTarget(speed,Pose2f(120_deg,0.f,0.f));
          WalkAtSpeedPercentage(Pose2f(0.7, 0.f, 0.f));
      }
  }


  state(sixth)
  {
      transition
      {
          if(state_time > 8000)
          {
              if(isMirror)
            {
               if(firstStep_flag == 4)
                  goto finish;
               else if(firstStep_flag == 3)
                  goto finish;
               else
                  goto finish;
            }
            else
            {
               if(firstStep_flag == 3)
                  goto finish;
                else if(firstStep_flag == 4)
                  goto finish;
                else
                  goto finish;
            }
          }
      }

      action
      {
          theHeadControlMode = HeadControl::lookLeftAndRightFast;
          Stand();
      }
  }

  target_state(finish)
  {
      action
      {
          inital_flag = 1;
          firstStep_flag = 1;
          // WalkToTarget(speed,Pose2f(0.f,0.f,0.f));  //@yy
          Stand();
      }
  }
}
