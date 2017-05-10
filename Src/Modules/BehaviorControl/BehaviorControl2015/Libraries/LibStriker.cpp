/**
 * @file LibStriker.cpp
 * @auther Deming Wang
 * @date Mar 15, 2017
 */

#include "../../BehaviorControl2015/LibraryBase.h"

#include <iostream>

namespace Behavior2015
{
#include "../../BehaviorControl2015/Libraries/LibRobot.h"
#include "../../BehaviorControl2015/Libraries/LibArea.h"
#include "../../BehaviorControl2015/Libraries/LibStriker.h"
#include "../../BehaviorControl2015/Libraries/LibObstacle.h"

  LibStriker::LibStriker():
        opponentNearestBall(5000.f,5000.f)
  
  {
  }

  void LibStriker::preProcess()
  {

    shootType = 0;
    shootArea = 0;
    centerObstacle = 0;
    centerArea[0] = 0;
    centerArea[1] = 0;
    near40CmOpponent.clear();
    for(int i = 0;i<3;i++)
    {
        opponentCountGoal[i]=0;
        opponentAngle[i] = 0.f;
        opponentDistance[i] = 0.f;
        opponentY[i] = 0.f;
        opponentX[i] = 0.f;
    }
    numObstacleNearby = 0;
    
  }

  void LibStriker::postProcess()
  {
  }

  int LibStriker::getNearestOpponent(float opponentDistance[])
  {
    int minIdx = 0;
    for (int i = 0; i < sizeof(opponentDistance)/sizeof(float) + 1; i++)  
    {  
//        std::cout<<"opponentDistance:"<<opponentDistance[i]<<std::endl;
        if(opponentDistance[i] == 0)
            opponentDistance[i] = 100000;
    }
    if(opponentDistance[0] < opponentDistance[1])
    {
        if(opponentDistance[0] < opponentDistance[2])
            minIdx = 0;
        else
            minIdx = 2;
    }
    else
    {
        if(opponentDistance[1] < opponentDistance[2])
            minIdx = 1;
        else
            minIdx = 2;
    }


//    std::cout<<"minDistanceOpponent:"<<minIdx<<"->"<<opponentDistance[minIdx]<<std::endl;

    return minIdx;
  }

int LibStriker::getObstacleInfo(Vector2f ballPositionField)
  {

    Vector2f goalCenter(4500.f,0.f);
    Vector2f goalLeft(4500.f,750.f);
    Vector2f goalRight(4500.f,-750.f);
    Vector2f goalLeftLine(4500.f,250.f);
    Vector2f goalRightLine(4500.f,-250.f);

    angleToGoalLeft = (goalLeft - ballPositionField).angle();
    angleToGoalCenter = (goalCenter - ballPositionField).angle();
    angleToGoalRight = (goalRight - ballPositionField).angle();
    angleToGoalLeftLine = (goalLeftLine - ballPositionField).angle();
    angleToGoalRightLine = (goalRightLine - ballPositionField).angle();
    float kickAngleRange = fabs(angleToGoalLeft - angleToGoalRight);

//    std::cout << "------------------------------------------" << std::endl;
//    std::cout << "angleToGoalLeft:" << angleToGoalLeft * 180.f / 3.1415926f<< std::endl;
//    std::cout << "------------------------------------------" << std::endl;
//    std::cout << "------------------------------------------" << std::endl;
//    std::cout << "angleToGoalCenter:"<< angleToGoalCenter * 180.f / 3.1415926f << std::endl;
//    std::cout << "------------------------------------------" << std::endl;
//    std::cout << "------------------------------------------" << std::endl;
//    std::cout << "angleToGoalRight:"<< angleToGoalRight * 180.f / 3.1415926f << std::endl;
//    std::cout << "------------------------------------------" << std::endl;
//    std::cout << "------------------------------------------" << std::endl;
//    std::cout << "angleToGoalLeftLine:"<< angleToGoalLeftLine * 180.f / 3.1415926f << std::endl;
//    std::cout << "------------------------------------------" << std::endl;
//    std::cout << "------------------------------------------" << std::endl;
//    std::cout << "angleToGoalRightLine:"<< angleToGoalRightLine * 180.f / 3.1415926f << std::endl;
//    std::cout << "------------------------------------------" << std::endl;
//    std::cout << "------------------------------------------" << std::endl;
//    std::cout << "kickAngleRange:"<< kickAngleRange * 180.f / 3.1415926f << std::endl;
//    std::cout << "------------------------------------------" << std::endl;
//    std::cout << "------------------------------------------" << std::endl;
//    std::cout << "ballPositionField.x():"<<ballPositionField.x()<<"ballPositionField.y()"<<ballPositionField.y()<< std::endl;
//    std::cout << "------------------------------------------" << std::endl;



    for (const Obstacle & obstacle : theObstacleModel.obstacles)
    {
        CIRCLE("module:Behavior2015:shootPath", obstacle.center.x(), obstacle.center.y(), 200.f,
        10, Drawings::solidPen, ColorRGBA::cyan, Drawings::noPen, ColorRGBA());
        CIRCLE("module:Behavior2015:shootPath", obstacle.center.x(), obstacle.center.y(), 20.f,
        30.f, Drawings::solidPen, ColorRGBA::red, Drawings::noPen, ColorRGBA());

//        std::cout<<theObstacleModel.obstacles.size()<<std::endl;
        if( obstacle.type != Obstacle::teammate ) //obstacle.type == Obstacle::opponent ||
        {
            Vector2f opponentOnField = Geometry::relative2FieldCoord(theRobotPose, obstacle.center);
            float opponentDistanceTemp = (opponentOnField - ballPositionField).norm();
            Angle opponentAngleTemp = (opponentOnField - ballPositionField).angle();

//            std::cout<<"opponent--x:"<<opponentOnField.x()<< "  " <<"opponent--y:"<<opponentOnField.y()<<std::endl;
//            std::cout<<"opponentDistance--> "<<opponentDistanceTemp<<std::endl;
//            std::cout<<"opponent--angle2:"<<opponentAngleTemp * 180.f / 3.1415926f<<std::endl;
            //球位于禁区线以内，并且角度不过于太斜
            if(opponentOnField.x() > ballPositionField.x() && ballPositionField.x() > 3200.f && ballPositionField.y() > -1000.f && ballPositionField.y() < 1000.f && opponentOnField.x() < 4400.f)
            {

                if(opponentOnField.y() > -1000.f && opponentOnField.y() < -250.f)
                {
                    opponentCountGoal[0]++;
                    if(opponentCountGoal[0] < 2 )
                    {
                        opponentDistance[0] = opponentDistanceTemp;
                        opponentY[0] = opponentOnField.y();
                        opponentX[0] = opponentOnField.x();
                    }
                    else if(opponentDistance[0] > opponentDistanceTemp)
                    {
                        opponentDistance[0] = opponentDistanceTemp;
                        opponentY[0] = opponentOnField.y();
                        opponentX[0] = opponentOnField.x();
                    }
                }
                else if(opponentOnField.y() > -250.f && opponentOnField.y() < 250.f)
                {
                    opponentCountGoal[1]++;
                    if(opponentCountGoal[1] < 2)
                    {
                        opponentDistance[1] = opponentDistanceTemp;
                        opponentY[1] = opponentOnField.y();
                        opponentX[1] = opponentOnField.x();
                    }
                    else if(opponentDistance[1] > opponentDistanceTemp)
                    {
                        opponentDistance[1] = opponentDistanceTemp;
                        opponentY[1] = opponentOnField.y();
                        opponentX[1] = opponentOnField.x();
                    }
                    if(opponentOnField.y() > 0)
                        centerArea[0]++;
                    else
                        centerArea[1]++;
                }
                else
                {
                    opponentCountGoal[2]++;
                    if(opponentCountGoal[2] < 2)
                    {
                        opponentDistance[2] = opponentDistanceTemp;
                        opponentY[2] = opponentOnField.y();
                        opponentX[2] = opponentOnField.x();
                    }
                    else if(opponentDistance[2] > opponentDistanceTemp)
                    {
                        opponentDistance[2] = opponentDistanceTemp;
                        opponentY[2] = opponentOnField.y();
                        opponentX[2] = opponentOnField.x();
                    }
                }
            }
            //球位于其他区域，并且障碍物在门柱夹角范围内
            else if( opponentOnField.x() > ballPositionField.x() && opponentOnField.x() < 4400.f && opponentAngleTemp > ( angleToGoalRight) && opponentAngleTemp < (angleToGoalLeft))
            {

                if(opponentAngleTemp > ( angleToGoalRight ) && opponentAngleTemp < angleToGoalRightLine)
                {
                    opponentCountGoal[0]++;
                    if(opponentAngle[0] == 0.f)
                    {
                        opponentAngle[0] = opponentAngleTemp;
                        opponentDistance[0] = opponentDistanceTemp;
                        opponentY[0] = opponentOnField.y();
                        opponentX[0] = opponentOnField.x();
                    }
                    else if(opponentDistance[0] > opponentDistanceTemp)
                    {
                        opponentAngle[0] = opponentAngleTemp;
                        opponentDistance[0] = opponentDistanceTemp;
                        opponentY[0] = opponentOnField.y();
                        opponentX[0] = opponentOnField.x();
                    }
                }
                else if(opponentAngleTemp > angleToGoalLeftLine && opponentAngleTemp < ( angleToGoalLeft ))
                {
                    opponentCountGoal[2]++;
                    if(opponentAngle[2] == 0.f)
                    {
                        opponentAngle[2] = opponentAngleTemp;
                        opponentDistance[2] = opponentDistanceTemp;
                        opponentY[2] = opponentOnField.y();
                        opponentX[2] = opponentOnField.x();
                    }
                    else if(opponentDistance[2] > opponentDistanceTemp)
                    {
                        opponentAngle[2] = opponentAngleTemp;
                        opponentDistance[2] = opponentDistanceTemp;
                        opponentY[2] = opponentOnField.y();
                        opponentX[2] = opponentOnField.x();
                    }
                }
                else
                {
                    opponentCountGoal[1]++;
                    centerObstacle = opponentAngleTemp > 0? 1:-1;
//                    std::cout<<"centerObstacle "<<centerObstacle<<std::endl;
                    if(opponentAngle[1] == 0.f)
                    {
                        opponentAngle[1] = opponentAngleTemp;
                        opponentDistance[1] = opponentDistanceTemp;
                        opponentY[1] = opponentOnField.y();
                        opponentX[1] = opponentOnField.x();
                    }
                    else if(opponentDistance[1] > opponentDistanceTemp)
                    {
                        opponentAngle[1] = opponentAngleTemp;
                        opponentDistance[1] = opponentDistanceTemp;
                        opponentY[1] = opponentOnField.y();
                        opponentX[1] = opponentOnField.x();
                    }
                }
            }
        }
    }


    if(ballPositionField.x() > 3200.f && ballPositionField.y() > -1000.f && ballPositionField.y() < 1000.f)
        return 1;
    else
        return 2;
  }
  int LibStriker:: getIntersection(Vector2f ballPositionField, Vector2f target, Vector2f & opponent)
  {
    int interSectionPoint = 0;

    float distance = std::numeric_limits<float>::max();
    float nearbyRadius = 750.0f;
    float d = 4500.f;


    //Copy from obstacle
    if (!theObstacleModel.obstacles.empty())
    {
      for (const Obstacle & obstacle : theObstacleModel.obstacles)
      {
        if(obstacle.type == Obstacle::goalpost )
          continue;
        if (obstacle.center.x() < 0)
          continue;
        d = (Geometry::relative2FieldCoord(theRobotPose, obstacle.center) - ballPositionField).norm();
//        std::cout<<"Distance----------------------------------------------->"<<d<<std::endl;
        if (d < nearbyRadius)
        {
          numObstacleNearby++;
        }
        if (d < distance && d < nearbyRadius)
        {
          distance = d;
          opponentNearestBall = obstacle.center;
        }
      }
      if(numObstacleNearby == 0)
        opponentNearestBall << 5000.f, 5000.f;
//      std::cout<<"numObstacleNearby---------------------------------------------->"<<numObstacleNearby<<std::endl;
//      std::cout<<"NearestDistance-------------------------------------------->"<<distance<<std::endl;
    }
    else
    {
      opponentNearestBall << 5000.f, 5000.f;
    }

    //std::cout<<"opponentNearestBall.x------------->"<<opponentNearestBall.x()<<" "<<"opponentNearestBall.y------------->"<<opponentNearestBall.y()<<std::endl;
    opponent = Geometry::relative2FieldCoord(theRobotPose, opponentNearestBall);
//    std::cout<<"opponentNearestBall.x------------->"<<opponent.x()<<" "<<"opponentNearestBall.y------------->"<<opponent.y()<<std::endl;

    // for(const Obstacle & obstacle : theObstacleModel.obstacles)
    // {
          CIRCLE("module:Behavior2015:shootPath", opponentNearestBall.x(), opponentNearestBall.y(), 220.f,
          10, Drawings::solidPen, ColorRGBA::yellow, Drawings::noPen, ColorRGBA());
          CIRCLE("module:Behavior2015:shootPath", opponentNearestBall.x(), opponentNearestBall.y(), 20.f,
          30.f, Drawings::solidPen, ColorRGBA::red, Drawings::noPen, ColorRGBA());

          Vector2f opponentOnFieldCircleCenter = Geometry::relative2FieldCoord(theRobotPose, opponentNearestBall);
          Geometry::Line line(ballPositionField, target - ballPositionField);
          Vector2f p1;
          Vector2f p2;
          Geometry::Circle obstacle_circle(opponentOnFieldCircleCenter,220.f);
          interSectionPoint = Geometry::getIntersectionOfLineAndCircle(line, obstacle_circle, p1, p2);
//          std::cout<<"getIntersectionOfLineAndCircle--------------------------->:::::"<<interSectionPoint<<std::endl;
    // }
//    std::cout<<"interSectionPoint-------------------------------------------->"<<interSectionPoint<<std::endl;
    return interSectionPoint;
  }
  bool LibStriker::getStrikerKickAngle(int &shootType, Vector2f ballPositionField, Vector2f &target)
  {

    //Initial Shoot Parameters
    shootOrNot = true;
    target.x() = 4500.f;
    int interSectionPoint = 0;
    //Get the 
    shootArea =  getObstacleInfo(ballPositionField);


    int minIdx = getNearestOpponent(opponentDistance);
    float shootRange = fabs(opponentAngle[1] - opponentAngle[0])*opponentDistance[minIdx];
//    std::cout<<"shootRange:"<<shootRange<<std::endl;
//    std::cout<<"AngleRang:"<<fabs(opponentAngle[1] - opponentAngle[0]) * 180.f / 3.1415926<<std::endl;


    //Get the ShootType 0~7
    for(int i = 0;i<3;i++)
    {
        if(opponentCountGoal[i]>0)
            shootType = shootType + pow(2,i);
//        std::cout<<"opponentCountGoal: "<<opponentCountGoal[i]<<std::endl;
    }
//    std::cout<<"shootType: "<<shootType<<std::endl;
    //如果靠近对方球门两侧底线直接朝着550.f或者-550.f的位置
    if(((ballPositionField.y() > 1500.f) || (ballPositionField.y() < -1500.f)) && ballPositionField.x() > 3200.f)
    {
        shootArea = 3;
        if(shootType == 0)
        {
        	target.y() = 0.f;
        	shootOrNot = true;
        }
        else
        {
        	target.y() = ballPositionField.y() > 1500.f? 500.f:-500.f;
            shootOrNot = false;
        }
    }
    else if((ballPositionField.x() > 0) && (ballPositionField.x() < 1000.f))
    {
        shootArea = 4;
//        std::cout<<"BallIsFarFromGoal!!!!!!!"<<std::endl;
    }
    if(shootArea == 1)//禁区前面的区域
    {
        switch(shootType)
        {
            case 0:
            {
                target.y() = 0.f;
                break;
            }
            case 1:
                {
                    target.x() = (4500.f + opponentX[0])/2;
                    target.y() = (750.f + opponentY[0])/2;
                    break;
                }
            case 2:
                {
                    target.x() = (4500.f + opponentX[1])/2;
                    if(ballPositionField.y() > opponentY[1] + 150.f)
                        target.y() = (750.f + opponentY[1])/2;
                    else if(ballPositionField.y() < opponentY[1] - 150.f)
                        target.y() = (-750.f + opponentY[1])/2;
                    else
                    {
                        target.y() = ballPositionField.y() > opponentY[1]?(750.f + opponentY[1])/2:(-750.f + opponentY[1])/2;
                        shootOrNot = false;
                        return shootOrNot;
                    }
                    break;
                }
            case 3:
            {
//                std::cout<<"opponent0 "<<opponentY[0]<<' '<<"opponent1 "<<opponentY[1]<<' '<<"opponent2"<<opponentY[2]<<std::endl;
                if(ballPositionField.y()>0)
                {
                    target.x() = (4500.f + opponentX[1])/2;
                    target.y() = (750.f + opponentY[1])/2;
                }
                else if( ballPositionField.y() > opponentY[1])
                {
                    target.x() = (4500.f + opponentX[1])/2;
                    target.y() = (750.f + opponentY[1])/2;
                }
                else if(ballPositionField.y() > opponentY[0])
                {
                    target.x() = ( opponentX[0] + opponentX[1])/2;
                    target.y() = ( opponentY[0] + opponentY[1])/2;
                }
                else if(ballPositionField.y() < -600.f)
                {
                    target.x() = ( opponentX[0] + 4500.f )/2;
                    target.y() = ( opponentY[0] - 750.f )/2;
                }
                else
                {
                    target.x() = ( opponentX[0] + opponentX[1])/2;
                    target.y() = ( opponentY[0] + opponentY[1])/2;
                }
                break;
            }
            case 4:
            {
                target.x() = (4500.f + opponentX[2])/2;
                target.y() = (-750.f + opponentY[2])/2;
                break;
            }
            case 5:
            {
                target.y() = 0.f;
                break;
            }
            case 6:
            {
                if(ballPositionField.y()<0)
                {
                    target.x() = (4500.f + opponentX[1])/2;
                    target.y() = (-750.f + opponentY[1])/2;
                }
                else if( ballPositionField.y() < opponentY[1])
                {
                    target.x() = (4500.f + opponentX[1])/2;
                    target.y() = (-750.f + opponentY[1])/2;
                }
                else if(ballPositionField.y() < opponentY[2])
                {
                    target.x() = ( opponentX[2] + opponentX[1])/2;
                    target.y() = ( opponentY[2] + opponentY[1])/2;
                }
                else if(ballPositionField.y() > 600.f)
                {
                    target.x() = ( opponentX[0] + 4500.f )/2;
                    target.y() = ( opponentY[2] + 850.f )/2;
                }
                else
                {
                    target.x() = ( opponentX[2] + opponentX[1])/2;
                    target.y() = ( opponentY[1] + opponentY[2])/2;
                }
                break;
            }
            case 7:
            {
                float intervalLeft = fabs(opponentY[2]-opponentY[1]);
                float intervalRight = fabs(opponentY[1]-opponentY[0]);
                if(ballPositionField.y() < 0.f)
                {
                    target.y() = (opponentY[1]+opponentY[0])/2;
                }
                else
                    target.y() = (opponentY[1]+opponentY[2])/2;
                break;
            }
            default:
            {
                target.y() = 0.f;
                break;
            }
        }
        // if(opponentDistance[minIdx] < 400.f)
        // {
        //     shootOrNot = false;
        //     return shootOrNot;
        // }
    }
    else if(shootArea == 2 || shootArea == 4)//罚球点以外的区域
    {
        switch(shootType)
        {
            case 0://射门区域内无障碍物的情况，直接射门
            {
                target.y() = 0.f;
                break;
            }
            case 1://障碍物呈001阵型
            {
                target.x() = opponentX[0];
                target.y() = ballPositionField.y()+(opponentX[0]-ballPositionField.x())*tan((opponentAngle[0]+angleToGoalLeft)/2) + 100.f;
                // if(opponentDistance[0] < 400.f)//障碍物距离太近的时候，进行dribble
                // {
                //     shootOrNot = false;
                //     return shootOrNot;
                // }
                break;
            }
            case 2://出现010情况
                {
                    target.y() = ballPositionField.y()>0?550.f:-550.f;
                    if(ballPositionField.y()> -200.f && ballPositionField.y() < 200.f)
                    {
                        target.x() = ( opponentX[1] + 4500.f ) / 2.0;
                        target.y() = centerObstacle>0?(opponentY[1]-750.f)/2.0 : (opponentY[1]+750.f)/2.0;
                    }
                    else if(opponentCountGoal[1] > 1 && ballPositionField.y() > 200.f)
                    {
                        target.x() = opponentX[1] + 500.f;
                        target.y() = opponentY[1] > ballPositionField.y()? (opponentY[1] - 500.f):(opponentY[1] + 500.f);
                        shootOrNot = false;
                        return shootOrNot;
                    }
                    else if(opponentCountGoal[1] > 1 && ballPositionField.y() < -200.f)
                    {
                        target.x() = opponentX[1] + 500.f;
                        target.y() = opponentY[1] > ballPositionField.y()? (opponentY[1] - 500.f):(opponentY[1] + 500.f);
                        shootOrNot = false;
                        return shootOrNot;
                    }
                    else if(angleToGoalLeft-opponentAngle[1]>opponentAngle[1]-angleToGoalRight)
                    {
                        target.x() = opponentX[1];
                        target.y() = ballPositionField.y()+(opponentX[1]-ballPositionField.x())*tan((opponentAngle[1]+angleToGoalLeft)/2) + 100.f;
//                        std::cout<<"LeftAngle"<<std::endl;
                    }
                    else if(angleToGoalLeft-opponentAngle[1]<opponentAngle[1]-angleToGoalRight)
                    {
                        target.x() = opponentX[1];
                        target.y() = ballPositionField.y()+(opponentX[1]-ballPositionField.x())*tan((opponentAngle[1]+angleToGoalRight)/2) - 100.f;
//                        std::cout<<"RightAngle"<<std::endl;
                    }
                    // if(opponentDistance[1] < 400.f)
                    // {
                    //     shootOrNot = false;
                    //     return shootOrNot;
                    // }
                    break;
                }
            case 3://出现011的情况
            {
                
                if ((ballPositionField.y()>opponentY[1]-100.f) && (ballPositionField.y() < opponentY[1] + 100.f))
                {
                    Vector2f opponentPosition(opponentX[1],opponentY[1]);
                    if((opponentPosition-ballPositionField).norm() < 500.f)
                    {
                        target.y() = ballPositionField.y() + (opponentX[1]-ballPositionField.x())*tan(angleToGoalCenter);
                        target.x() = opponentX[1] + 500.f;
                        shootOrNot =  false;
                    }
                    else
                    {
                        target.y() = ballPositionField.y() + (opponentX[1]-ballPositionField.x())*tan((opponentAngle[1] + angleToGoalLeft)/2) + 100.f;
                        target.x() = opponentX[1];
                        shootOrNot = true;
                    }
                    return shootOrNot;
                }
                else if(ballPositionField.y() > opponentY[1] + 100.f )//球位于场地的左边
                {
                    // if(opponentAngle[1] < angleToGoalCenter && ballPositionField.y() > opponentAngle[1])//中间障碍物位于中线的右边并且球位于障碍物的左边
                    // {
                        //朝着左门柱和中线的角平分线处踢球，若坐标点超出750，则往中间dribble
                        target.y() = ballPositionField.y() + (opponentX[1]-ballPositionField.x())*tan((opponentAngle[1] + angleToGoalLeft)/2) + 100.f;
                        target.x() = opponentX[1];
                    // }
                    // //其他情况往两者中间踢
                    // else
                    // {
                    //  target.x() = ( opponentX[0] + opponentX[1])/2;
                    //  target.y() = ( opponentY[0] + opponentY[1])/2;
                    // }
                }//球位于场地的右边，在400以内
                else if(ballPositionField.y() > -400.f || ballPositionField.y() > opponentY[0])
                {
                    if(shootRange > 250.f)
                    {
                        target.x() = ( opponentX[0] + opponentX[1])/2;
                        target.y() = ( opponentY[0] + opponentY[1])/2;
                    }
                    else
                    {
                        // target.x() = ballPositionField.x() + 500.f;
                        target.x() = opponentX[minIdx] + 500.f;
                        target.y() = ballPositionField.y() + 500.f * tan(angleToGoalCenter);
//                        std::cout<<"HHHHHHHHHERERERER"<<std::endl;
                        shootOrNot = false;
                        return shootOrNot;
                    }

                }
                else if (ballPositionField.y() > -750.f && opponentY[1] < 0.f)
                {
                    target.y() = ballPositionField.y() + (opponentX[1]-ballPositionField.x())*tan((opponentAngle[1] + angleToGoalLeft)/2) + 100.f;
                    target.x() = opponentX[1];
                }
                else
                {
                    target.x() = ballPositionField.x() + 500.f;
                    target.y() = ballPositionField.y() + 500.f * tan(angleToGoalCenter);
                    shootOrNot = false;
                    return shootOrNot;
                }
                // if(opponentDistance[minIdx] < 400.f)
                // {
                //     shootOrNot = false;
                //     return shootOrNot;
                // }
                break;
            }
            case 4:
            {
                target.x() = opponentX[2];
                target.y() = ballPositionField.y()+(opponentX[2]-ballPositionField.x())*tan((opponentAngle[2]+angleToGoalRight)/2)- 100.f;
                // if(opponentDistance[2] < 400.f)
                // {
                //     shootOrNot = false;
                //     return shootOrNot;
                // }
                break;
            }
            case 5:
            {
                target.x() = (opponentX[0] + opponentX[2]) / 2;
            	target.y() = (opponentY[0] + opponentY[2]) / 2;
                // if(fabs(ballPositionField.y())>750.f)
                // {
                //     target.x() = ballPositionField.x() + 500.f;
                //     target.y() = 0.f;
                //     shootOrNot = false;
                //     return shootOrNot;
                // }
                break;
            }
            case 6:
            {
                if ((ballPositionField.y()>opponentY[1]-100.f) && (ballPositionField.y() < opponentY[1] + 100.f))
                {
                    Vector2f opponentPosition(opponentX[1],opponentY[1]);
                    if((opponentPosition-ballPositionField).norm() < 500.f)
                    {
                        target.y() = ballPositionField.y() + (opponentX[1]-ballPositionField.x())*tan(angleToGoalCenter);
                        target.x() = opponentX[1] + 500.f;
                        shootOrNot =  false;
                    }
                    else
                    {
                        target.y() = ballPositionField.y() + (opponentX[1]-ballPositionField.x())*tan((opponentAngle[1] + angleToGoalRight)/2) - 100.f;
                        target.x() = opponentX[1];
                        shootOrNot = true;
                    }
                    return shootOrNot;
                }
                else if(ballPositionField.y() < opponentY[1] - 100.f )//
                {
                    target.y() = ballPositionField.y() + (opponentX[1]-ballPositionField.x())*tan((opponentAngle[1] + angleToGoalRight)/2) - 100.f;
                    target.x() = opponentX[1];
                }//球位于场地的右边，在400以内
                else if(ballPositionField.y() < 400.f || ballPositionField.y() < opponentY[2])
                {
                    if(fabs(opponentAngle[1]-opponentAngle[0]) > 15_deg)
                    {
                        target.x() = ( opponentX[2] + opponentX[1])/2;
                        target.y() = ( opponentY[1] + opponentY[2])/2;
                    }
                    else
                    {
                        target.x() = ballPositionField.x() + 500.f;
                        target.y() = ballPositionField.y() + 500.f * tan(angleToGoalCenter);
                        shootOrNot = false;
                        return shootOrNot;
                    }
                }
                else if(ballPositionField.y() < 750.f && opponentY[1] > 0.f)
                {
                    target.x() = opponentX[1];
                    target.y() = ballPositionField.y()+(opponentX[1]-ballPositionField.x())*tan((opponentAngle[1]+angleToGoalRight)/2)- 100.f;
                }
                else
                {
                    target.x() = ballPositionField.x() + 500.f;
                    target.y() = ballPositionField.y() + 500.f * tan(angleToGoalCenter);
                    shootOrNot = false;
                    return shootOrNot;
                }
                // if(opponentDistance[minIdx] < 400.f)
                // {
                //     shootOrNot = false;
                //     return shootOrNot;
                // }
                break;
            }
            case 7:
            {
                float intervalLeft = fabs(opponentAngle[2]-opponentAngle[1]);
                float intervalRight = fabs(opponentAngle[1]-opponentAngle[0]);
                if(intervalRight > intervalLeft)
                {

                    target.y() = ballPositionField.y() + (4500.f-ballPositionField.x())*tan((opponentAngle[1]+opponentAngle[0])/2);
                }
                else
                    target.y() = ballPositionField.y() + (4500.f-ballPositionField.x())*tan((opponentAngle[1]+opponentAngle[2])/2);
                break;
            }
            default:
            {
                target.y() = 0.f;
                break;
            }
        }
    }
    
    // CIRCLE("module:Behavior2015:shootPath", obstacle.center.x(), obstacle.center.y(), 200.f,
    // 10, Drawings::solidPen, ColorRGBA::yellow, Drawings::noPen, ColorRGBA());
    // CIRCLE("module:Behavior2015:shootPath", obstacle.center.x(), obstacle.center.y(), 20.f,
    // 30.f, Drawings::solidPen, ColorRGBA::red, Drawings::noPen, ColorRGBA());
    return shootOrNot;
  }

}
