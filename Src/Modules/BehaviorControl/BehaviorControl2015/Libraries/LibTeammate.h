/**
 * @file LibTeammate.h
 * @auther Hanbin Wang
 * @date Mar 11, 2014
 */

#pragma once
struct RoleData
{
  bool isOnline = false;
  bool isPenalized = false;

  Pose2f pose;
  float ballDistance;
};
class LibTeammate : public LibraryBase
{
 public:
  /** Constructor for initializing all members*/
  LibTeammate();

  void preProcess() override;
  void postProcess() override;

  //float x;
  //@Usage: mate.robotPose(Your teammate's number)
  //@param playerNum the number of the teammate: 1-5

  Pose2f robotPose(int playerNum);
  Teammate::Status robotState(int playNum);
  Vector2f ballGlobalFromRobot(int playNum);
  Vector2f ballRobotFromRobot(int playNum);
  int getAreaNum(float robotX, float robotY);
  int getSupporterCurrentAreaNum();
  int getStrikerAreaNum();
  int getSupporterNextAreaNum();
  Pose2f getSupporterPose();

  RoleData keeper,striker,supporter,defender,stabber;
  void setRoleData(RoleData &role, const Teammate &teammate);
  void setRoleData(RoleData &role);
//  bool keeperKick;
//  bool defenderKick;
//  bool strikerKick;
//  bool supporterKick;
//  bool stabberKick;

//  bool isKeeperPenalized;
//  bool isStabberReadyForKickOff();

//  bool isOtherTeammatesControlBall;

};
