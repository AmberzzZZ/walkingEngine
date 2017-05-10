/**
* @file RoleProvider.h
* This file declares a module that updates the role of the robot
* @author xiong
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
//@YANG
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
//#include "Representations/Infrastructure/SensorData/UsSensorData.h"B

#include "Representations/Communication/TeammateData.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/FieldPercepts/GoalPercept2017.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Sensing/GroundContactState.h"
//#include "Representations/BehaviorControl/BehaviorControlOutput.h"
//#include "Representations/BehaviorControl/TimeToReachBall.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/RoleBehavior.h"
//#include "Representations/Configuration/ReadyPose.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"

#include "Representations/Configuration/HeadLimits.h"

MODULE(RoleProvider,
{
  USES(BehaviorControlOutput)  ,
  USES(MotionRequest) ,
  USES(MotionInfo) ,
  USES(RoleBehavior) ,
  REQUIRES(FrameInfo) ,
  REQUIRES(GameInfo) ,
  REQUIRES(RobotInfo) ,
  REQUIRES(OwnTeamInfo) ,
  REQUIRES(BallModel) ,
  REQUIRES(RobotPose) ,
  REQUIRES(ObstacleModel) ,
  REQUIRES(TeammateData) ,
  REQUIRES(GroundContactState) ,
 // REQUIRES(TimeToReachBall) ,
  PROVIDES(Role) ,
});

class RoleProvider : public RoleProviderBase
{
private:
//  int preGameState;
  
  void update(Role& role);
  
  /*role init when the robot get set or return to the field*/
  
  void initial(Role& role);

  void getRoleDataOnline(Role& role);

  void resetRequestType(Role& role);

  void checkRoleAssign(Role& role);

//  void stateUpdate(Role& role);
  /*Every robot caculates its time to get to the ball and make decision about
   *being a striker.Striker's decision has the highest priority*/
  bool strikerDecision(Role& role,int robotNum = 0);
  bool supporterDecision(Role& role,int robotNum = 0);
  bool defenderDecision(Role& role,int robotNum = 0);
  bool stabberDecision(Role& role,int robotNum = 0);

  void strikerRoleAssignDecision(Role &role);
  void supporterRoleAssignDecision(Role &role);
  void defenderRoleAssignDecision(Role &role);
  void stabberRoleAssignDecision(Role &role);

  bool requestRole(Role &role, Role::RoleType roleType);
  void changeRoleIfReceiveRequest(Role& role);

//  void isKickOffByStriker(Role& role);
//  void isStabberReady(Role& role);
//  void getKickOffChoiceByStriker(Role& role);

  int numOnline = 0;

//  int gameState = 0;
//  int lastGameState = 0;

  public:
  RoleProvider();
};

