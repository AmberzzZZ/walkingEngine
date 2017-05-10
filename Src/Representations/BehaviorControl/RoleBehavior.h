/**
 * @file Representations/BehaviorControl/RoleBehavior.h
 *
 * Update on strategy
 *
 * @author WZF
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Role.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

STREAMABLE(KeeperBehavior,
{
  void draw() const,

  (bool)(false) valid,
  (bool)(false) isControlBall,
});

STREAMABLE(StrikerBehavior,
{
  ENUM(KickOffDirection,
  {,
    LEFT,
    MIDDLE,
    RIGHT,
  });

//  ENUM(RoleType,
//  {,
//    none,
//    keeper,
//    striker,
//    supporter,
//    defender,
//    stabber,
//    requestNotAllow,
//    undefined,
//  });

  void draw() const,

  (bool)(false) valid,
  (bool)(false) isRequestNotAllowed,
//  (RoleType)(none) requestRoleType,
  ((Role) RoleType)(Role::none) requestRoleType,
  (bool)(false) isControlBall,

  (bool)(false) readyAfterPlaying,
  (bool)(false) isKickOffLimit,
  (KickOffDirection)(MIDDLE) kickOffDirection,
});

STREAMABLE(SupporterBehavior,
{
  ENUM(KickOffDirection,
  {,
    LEFT,
    MIDDLE,
    RIGHT,
  });
  void draw() const,

  (bool)(false) valid,
  (bool)(false) isRequestNotAllowed,
  ((Role) RoleType)(Role::none) requestRoleType,
  (bool)(false) isControlBall,

  (KickOffDirection)(MIDDLE) kickOffDirection,

});

STREAMABLE(DefenderBehavior,
{
  void draw() const,

  (bool)(false) valid,
  (bool)(false) isRequestNotAllowed,
  ((Role) RoleType)(Role::none) requestRoleType,
  (bool)(false) isControlBall,
});

STREAMABLE(StabberBehavior,
{
  void draw() const,

  (bool)(false) valid,
  (bool)(false) isRequestNotAllowed,
  ((Role) RoleType)(Role::none) requestRoleType,
  (bool)(false) isControlBall,
});

STREAMABLE(RoleBehavior,
{
  void draw() const,
  (KeeperBehavior) keeperBehavior,
  (StrikerBehavior) strikerBehavior,
  (SupporterBehavior) supporterBehavior,
  (DefenderBehavior) defenderBehavior,
  (StabberBehavior) stabberBehavior,
});
