/**
 * @file Representations/BehaviorControl/Role.h
 *
 * Declaration of the the representation of a robot's behavior role
 *
 * @author Tim Laue, Andreas Stolpmann
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

/**
 * @struct Role
 * Representation of a robot's behavior role
 */
STREAMABLE(Role,
{
  ENUM(KickOffChoice,
  {,
    left,
    middle,
    right,
  });

  /** The different roles */
  // 0-5 的顺序不能改！  @wzf
  ENUM(RoleType,
  {,
    none,
    keeper,
    striker,
    supporter,
    defender,
    stabber,
    requestNotAllow,
    undefined,
  });

  /** Draws the current role next to the robot on the field view (in local robot coordinates) */
  void draw() const,

  /** Instance of role */
  (RoleType)(striker) role,
  (RoleType)(none) requestType,
  (bool)(false) isRequestNotAllowed,
  (bool)(false) wasPenalized,

  (std::vector<int>) currentKeeper,
  (std::vector<int>) currentStriker,
  (std::vector<int>) currentSupporter,
  (std::vector<int>) currentDefender,
  (std::vector<int>) currentStabber,
  (bool)(true) isKeeperOnline,
  (bool)(true) isStrikerOnline,
  (bool)(true) isSupporterOnline,
  (bool)(true) isDefenderOnline,
  (bool)(true) isStabberOnline,

   /*the Robot's preview role.
    *if requestType is 0, it keeps to the current role.
    *otherwise it is the same to the role before requesType changing to
    *none-zero*/
  (RoleType)(striker) previewR,

  /*The timeStamp the of the last roleChange.
   *The role should last at least 10s*/
  (unsigned)(0) lastRequestTime,

});

STREAMABLE(TeammateRoles,
{
  Role::RoleType operator [] (const size_t i) const;
  Role::RoleType& operator [] (const size_t i);
  static const char* getName(Role::RoleType e),

  (std::vector<Role::RoleType>) roles,
});

STREAMABLE(RoleCompressed,
{
  RoleCompressed() = default;
  RoleCompressed(const Role & role_);
  operator Role() const,

  ((Role) RoleType) role,
  (bool) wasPenalized,
  ((Role) RoleType) requestType,
  (bool)(false) isRequestNotAllowed,
  ((Role) RoleType) previewR,
});
