/**
 * @file LibStriker.h
 * @auther Deming Wang
 * @date Mar 15, 2017
 */

#pragma once

class LibStriker : public LibraryBase
{
 public:
  /** Constructor for initializing all members*/
  LibStriker();


  void preProcess() override;
  void postProcess() override;


  //Used For Striker Kick

  int opponentCountGoal[3] = {0,0,0};//Three Area Count
  float opponentAngle[3] = {0,0,0};//Three Area Opponent Angle
  float opponentDistance[3] = {0,0,0};//Three Area Opponent Angle Used for selecting nearest opponent
  float opponentY[3] = {0,0,0};//Three Area Opponent Y
  float opponentX[3] = {0,0,0};//Three Area Opponent Y
  int shootType ;
  int shootArea ;// 0-default 1-near(点球点到禁区前的区域）2-中间区域 3-靠近对方球门两侧底线的区域 4-两侧边线附近
  int centerObstacle;
  int centerArea[2];
  bool lastFoot = false;
  bool shootOrNot = true;
  std::vector<Vector2f> near40CmOpponent;
  Vector2f opponentNearestBall;
  int numObstacleNearby;


  Angle angleToGoalLeft ;
  Angle angleToGoalCenter ;
  Angle angleToGoalRight;
  Angle angleToGoalLeftLine ;
  Angle angleToGoalRightLine ;



  bool getStrikerKickAngle(int &shootType,Vector2f ballPositionField, Vector2f &target);
  int getObstacleInfo(Vector2f ballPositionField);
  int getNearestOpponent(float opponentDistance[]);
  int getIntersection(Vector2f ballPositionField, Vector2f target, Vector2f & opponent);



};
