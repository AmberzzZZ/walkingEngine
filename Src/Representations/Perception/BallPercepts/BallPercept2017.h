/**
 * @file BallPercept.h
 *
 * Very simple representation of a seen ball
 *
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 * @autor Jesse
 */

#pragma once

#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include <vector>

STREAMABLE(BallPercept2017,
{
  STREAMABLE(BallRegions,
  {,
	  (Vector2i) center,
	  (float) radius,
	  (float) expectedRadius,
	  (int) avgGreen,
	  (float) greenRatio,
	  (int) greenInCircle,
	  (bool) patternCheck,
  });
  
  ENUM(Status,
  {,
    notSeen, /**< The ball was not seen. */
    seen,    /**< The ball was seen. */
    guessed, /**< Would be ok for a moving ball. */
  });

  BallPercept2017() = default;
  inline BallPercept2017(const Vector2f& positionInImage, const float radiusInImage, const Vector2f& relativePositionOnField, const float radiusOnField, const BallPercept2017::Status status);

  /** Draws the ball*/
  void draw() const,

  (Vector2f) positionInImage,         /**< The position of the ball in the current image */
  (float) radiusInImage,              /**< The radius of the ball in the current image */
  (Status)(notSeen) status,           /**< Indicates, if the ball was seen in the current image. */
  (Vector2f) positionOnField,         /**< Ball position relative to the robot. */
  (float)(50.f) radiusOnField,        /**< The radius of the ball on the field in mm */
  (float)(0) validity,                /**< The validity of the ball percept. */
  
  
  (std::vector<BallRegions>) candidateBalls,
});

inline BallPercept2017::BallPercept2017(const Vector2f& positionInImage, const float radiusInImage, const Vector2f& positionOnField, const float radiusOnField, const BallPercept2017::Status status = BallPercept2017::Status::seen) :
  positionInImage(positionInImage), radiusInImage(radiusInImage), status(status), positionOnField(positionOnField), radiusOnField(radiusOnField) {}
