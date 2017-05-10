/**
 * @
 * @author WZF
 * @date May 8th, 2016
 *
 */
#pragma once

class LibBehavior : public LibraryBase
{
 public:
  LibBehavior();

  void preProcess() override;
  void postProcess() override;

  void init();
  void getRoleBehavior();
  void resetOwnRoleBehavior();
  void updateOwnRoleBehavior();
  void updateSPLStandardBehaviorStatus();

  /** Messages from other teammates*/
  KeeperBehavior keeper;
  StrikerBehavior striker;
  SupporterBehavior supporter;
  DefenderBehavior defender;
  StabberBehavior stabber;

  bool othersControlBall = false;

  /** Messages prepared to broadcast, just use own role's behavior*/
  KeeperBehavior keeperOutput;
  StrikerBehavior strikerOutput;
  SupporterBehavior supporterOutput;
  DefenderBehavior defenderOutput;
  StabberBehavior stabberOutput;

  bool readyAfterPlaying = false;
  void roleRequestWhenReadyState();
  void roleRequestWhenReadyState(Role::RoleType role);

  bool isOtherTeammateControlBall();
  float timeToReachBall();
};
