/*
 * LibObstacle.cpp
 *
 *  Created on: Nov 3, 2015
 *      Author: yongqi
 */
#include "../../BehaviorControl2015/LibraryBase.h"

namespace Behavior2015
{
#include "../../BehaviorControl2015/Libraries/LibObstacle.h"

  LibObstacle::LibObstacle()
      : distance(0.f),
        center(3000.f, 1000.f),
        left(3000.f, 1000.f),
        right(3000.f, 1000.f)
  {
  }

  void LibObstacle::preProcess()
  {
    distance = std::numeric_limits<float>::max();
    numObstacleNearby = 0;
    float nearbyRadius = 700.0f;
    float d = 4500;
//    std::cout<<timeSinceOppKeeperWasSeen<<std::endl;
    if(timeSinceOppKeeperWasSeen < 100000)
      timeSinceOppKeeperWasSeen += 10;
    if (!theObstacleModel.obstacles.empty())
    {
      for (const Obstacle & obstacle : theObstacleModel.obstacles)
      {
        if(obstacle.type == Obstacle::goalpost || obstacle.type == Obstacle::unknown)
          continue;
        if (obstacle.center.x() < 0)
          continue;
        d = obstacle.center.norm();
        if (d < nearbyRadius)
        {
          numObstacleNearby++;
        }
        if (d < distance)
        {
          distance = d;
          center = obstacle.center;
          left = obstacle.left;
          right = obstacle.right;
        }
      }
      for (const Obstacle & obstacle : theObstacleModel.obstacles)
      {
        if(obstacle.type == Obstacle::opponent || obstacle.type == Obstacle::teammate || obstacle.type == Obstacle::someRobot)
        {
          Pose2f poseOfOppKeeper = Geometry::relative2FieldCoord(theRobotPose,obstacle.center);
          if(fabs(poseOfOppKeeper.translation.x() - 4200.f) < 800 && fabs(poseOfOppKeeper.translation.y()) < 1000)
          {
            timeSinceOppKeeperWasSeen = 0;
            oppKeeper = obstacle.center;
            oppKeeperDistance = oppKeeper.norm();
          }
        }
      }
    }
    else
    {
      center << 3000.f, 1000.f;
      left << 3000.f, 1000.f;
      right << 3000.f, 1000.f;
    }
  }

  void LibObstacle::postProcess()
  {
  }

}
