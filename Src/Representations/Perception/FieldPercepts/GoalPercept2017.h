/**
* @file GoalPercept2017.h
*
* representation of a seen goal
*
* @author Li Shu
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/Enum.h"
#include "stdint.h"

STREAMABLE(GoalPercept2017,
{
  STREAMABLE(GoalPost,
  {
    /** Defines the direction of goal posts */
    ENUM(GoalPostSide,
    {,
      leftPost,
      rightPost,
      unknownPost,
    }),

    (Vector2f) locationOnField,
    (Vector2i) bottomInImage, 					// bottom middle point of post
    (Vector2i) topInImage,						// top middle point of post
    (float) topWidth, 							// top Width of goalpost (horizontal)
    (float) bottomWidth, 						// bottom Width of goalpost (horizontal)
    (GoalPostSide)(unknownPost) goalPostSide,
    (bool) bottomFound, 						// is field found below goal post
    (bool) foundLineAtBottom, 					// is field line found at goal post bottom
    (bool) fromUpper, 							// found in upper cam ?
    (float) validity,
  });

  /** Constructor */
  GoalPercept2017() { goalPosts.reserve(8); }

  /** Draws the goal */
  void draw() const,

  (std::vector<GoalPost>) goalPosts,
  (int)(0) numberOfGoalPosts,
  (unsigned)(0) timeWhenLastSeen,
});

