/**
 * @file ICPPoseHypothesis.h
 *
 * A new robot pose, based on recently observed field features.
 * This pose is meant to be used for sensor resetting inside the SelfLocalization
 *
 * @author Zeng Zhiying
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"

/**
 * @struct AlternativeRobotPoseHypothesis
 * An alternative pose of the robot
 */
STREAMABLE(ICPPoseHypothesis,
{
  /** Draws this representation. */
  void draw() const,

  (Pose2f) pose,                             /** the pose in 2D (position + rotation) */
  (bool)(false) isValid,                     /** true, if the content of pose is valid */
  (unsigned)(0) timeOfLastIterateUpdate,  /** point of time, when the last perception that contributed to the pose was seen*/
  (bool)(false) isInOwnHalf,                 /** true, if the pose is probably in the own half */
});
