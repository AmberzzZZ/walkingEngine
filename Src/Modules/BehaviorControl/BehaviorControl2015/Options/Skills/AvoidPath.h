option(AvoidPath,(const Pose2f&) pose)
{
    float obsWidth = 250.f;//obsWidth is  obstacles' width
//    if((ball.distance<1000)||((sqr(robot.x-pose.translation.x())+sqr(robot.y-pose.translation.y()))<sqr(1000)))
//        obsWidth = 500.f;
//    if((fabs(ball.x-obstacle.x)<400)||(fabs(Geometry::fieldCoord2Relative(theRobotPose,Vector2f(pose.translation.x(),pose.translation.y())).x()-obstacle.x)<400))
//        obsWidth = 300;
//    #ifdef TARGET_SIM
//        obsWidth = 330.f;
//        if((ball.distance<1000)||((sqr(robot.x-pose.translation.x())+sqr(robot.y-pose.translation.y()))<sqr(1000)))//1200 for debug
//            obsWidth = 250.f;
//        if((fabs(ball.x-obstacle.x)<400)||(fabs(Geometry::fieldCoord2Relative(theRobotPose,Vector2f(pose.translation.x(),pose.translation.y())).x()-obstacle.x)<400))
//            obsWidth = 150;
//    #endif
   	Vector2f globalObstacle = Geometry::relative2FieldCoord(Pose2f(robot.rotation,robot.x,robot.y),obstacle.x,obstacle.y);
    Vector2f poseToRobot = robot.pose.translation-pose.translation;
    Vector2f poseToObstacle = globalObstacle-pose.translation;
    float angleDiff = fabs(poseToRobot.angle()-poseToObstacle.angle());
//    std::cout<<"globalObstacle = "<<globalObstacle.x()<<" ,"<<globalObstacle.y()<<std::endl;
//    std::cout<<"poseToRobot.angle : "<<poseToRobot.angle()<<",  poseToObstacle.angle : "<<poseToObstacle.angle()<<std::endl;
//    std::cout<<std::endl;
    initial_state(initial)
    {
        transition
        {
//        	std::cout<<"test"<<std::endl;
//        	PlaySound("doh.wav");
            //if(walk.toAvoidFar&&(pose.translation-walk.nearestObs).abs()>800)
            if(obstacle.x>500 && obstacle.x<1200 && angleDiff<1.0/*fabs(obstacle.y)<obsWidth /*&& fabs(obstacle.y)>0 &&
            		(pose.translation-Geometry::relative2FieldCoord(Pose2f(robot.rotation,robot.x,robot.y),obstacle.x,obstacle.y)).norm()>800*/)
            {
            	goto goToPointInPath;

            }
            else if(state_time>500)
           	{
            	goto avoid_success;

           	}
        }
    }

    state(goToPointInPath)
    {
    	static bool flag = false;

        transition
        {
            //if(!walk.toAvoidFar)
            if(angleDiff>1.0/*fabs(obstacle.y)>obsWidth && action_done*/)
            {
            	goto avoid_success;

            }
            else if(state_time>5000)
            {
            	obstacle.x=0;
            	obstacle.y=0;
            	goto avoid_success;
            }
        }
        action
        {
            static Vector2f point;
            if(!flag)
            {
            	point=walk.findPath(Vector2f(pose.translation.x(),pose.translation.y()));
            	flag =true;
            }
            Vector2f relativePoint = theRobotPose.inverse() * (point); //fieldCoord2Relative.@tjark_hjq
            Vector2f filterPoint;
            if(obstacle.y<0)
                if(relativePoint.y()>0)
                	filterPoint = relativePoint;
                else
                	filterPoint = Vector2f(100.f,100.f);
            else
            	if(relativePoint.y()<0)
                	filterPoint = relativePoint;
                else
                	filterPoint = Vector2f(100.f,-100.f);

            DECLARE_DEBUG_DRAWING("representation:Role", "drawingOnField");
            //WalkToPose(Pose2f(pose.rotation,point));
            //std::cout<<filterPoint.x()<<"xxxxxxxxxxxxx"<<filterPoint.y()<<std::endl;
//            WalkToTarget(Pose2f(0.8f,0.8f,0.8f),Pose2f(pose.rotation,filterPoint.x(),filterPoint.y()));
//            WalkToTarget(Pose2f(0.8f,0.4f,0.5f),Pose2f(pose.rotation,filterPoint.x(),filterPoint.y()));
            Vector2f globalFilterPoint = Geometry::relative2FieldCoord(Pose2f(robot.rotation,robot.x,robot.y),filterPoint.x(),filterPoint.y());
            CROSS("representation:Role", globalFilterPoint.x(), globalFilterPoint.y(), 50, 45, Drawings::solidPen, ColorRGBA(0xff, 0, 0));
            if (filterPoint.norm() < 200)
            	WalkToTarget(Pose2f(0.8f,0.4f,0.5f),Pose2f(pose.rotation,filterPoint.x(),filterPoint.y()));
            else
            	WalkToPoint(Pose2f(pose.rotation,globalFilterPoint));

            if(state_time % 1000 == 0)
            	flag =false;
        }
    }

    target_state(avoid_success)
    {
        transition
        {
            goto initial;
        }
    }
}
