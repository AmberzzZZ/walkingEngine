/************************************************
 * @file   LibPlayer.h
 * @author XiangWei Wang
 * @date   2014 25th Mar
 * @usage  PlayerParameter
 ************************************************/

#pragma once

class LibPlayer : public LibraryBase
{
 public:
  LibPlayer();

  void preProcess() override;
  void postProcess() override;

  float angleInPenalty;
  float angleInPenaltyLeft;
  float angleInPenaltyRight;
  float angleInKickBackward;
  //float xKeeper;
  float yKeeper;
  float yOffset;
  float xBall;
  float yBall;
  float LOM;			//left or middle line
  float MOR;	//right or middle line   these two are used to decide whether the ball is in the middle, left or right zone.
  float defendLine;
  float kickLine;
  float ballAngleRad;
  int random;
  float lastSeenPosition;
  Vector2f target;
  Vector2f targetLeft;
  Vector2f targetMid;
  Vector2f targetRight;
  Vector2f defendPosition;

  bool dynamicRoleAssign;

  bool strikerKickOff;		//@han: kick-off team
  unsigned playTime;

};
