/**
 * @file LibRobot.h
 * @auther Hanbin Wang
 * @date Feb 27, 2014
 */

#pragma once

class LibRobot : public LibraryBase
{
 public:
  /** Constructor for initializing all members*/
  LibRobot();


  void preProcess() override;
  void postProcess() override;

  ENUM(KickOffType,
  {,
	DEFAULT,
    BOTHIN,
    BOTHOUT,
    STRIKERIN,
	STABBERIN,
  });

  Pose2f pose;
  float &x = pose.translation.x();
  float &y = pose.translation.y();
  float &rotation = pose.rotation;  //Rad
  float angleToGoal;

  int numKeeper;		//#1
  int numStriker;		//#2
  int numSupporter;		//#3
  int numDefender;		//#4
  int numStabber;		//#5

  Pose2f odometry;

  //Used for supporter kickoff
  float kickOffAngle = 0.f;
  float kickForce = 0.75f;
  int kickOffDirection = 0;
  Vector2f kickOffTargetPoint;
  int opponentNearCount = 0;
  int opponentCount[6] = {0,0,0,0,0,0};
  int opponentFarCount[6] = {0,0,0,0,0,0};
  float opponentKickoffAngle[6] = {0,0,0,0,0,0};

  KickOffType kickOffType = DEFAULT;
  bool dribbleKickOff = false;
  bool isKickOffTypeConfirm = false;

  std::vector<int> oppoIndexArea1;
  std::vector<int> oppoIndexArea2;
  std::vector<int> oppoIndexArea3;
  std::vector<int> oppoIndexArea4;
  std::vector<int> oppoIndexArea5;
  std::vector<int> oppoIndexArea6;



  const Pose2f keeperReadyPose;
  const Pose2f strikerReadyPose;
  const Pose2f strikerReadyPoseWhenKickOff;
  const Pose2f supporterReadyPose;
  const Pose2f supporterReadyPoseWhenKickOff;
  const Pose2f defenderReadyPose;
  const Pose2f stabberReadyPose;


  bool lastFoot = false;
  Pose2f target_last =  Pose2f(0.f,0.f,0.f);



  bool isFallen();
  bool isNearOppGoalLeft();
  bool isNearOppGoalRight();
  bool isNearOppGoalMiddle();
  void setLearning(bool learn);
//
  int sumOpponentNearLeft();
  int sumOpponentNearRight();
  int sumOpponentFarLeft();
  int sumOpponentFarRight();




};
