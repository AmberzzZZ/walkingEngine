/*
 * LibObstacle.h
 *
 *  Created on: Nov 3, 2015
 *      Author: yongqi
 *  @TODO provide obstacle's size?
 */

#pragma once

class LibObstacle : public LibraryBase
{
 public:
  LibObstacle();

  void preProcess() override;
  void postProcess() override;

  float distance;
  Vector2f center;
  Vector2f left;
  Vector2f right;

  Vector2f oppKeeper;
  float oppKeeperDistance;
  int timeSinceOppKeeperWasSeen = 0;
  int numObstacleNearby = 0;//the number of obstacles nearby

  float &x = center.x();
  float &y = center.y();
  float &leftY = left.y();
  float &rightY = right.y();
};
