/**
 * @file GoalDetect2017.h
 *
 * Representation of a seen goal
 *
 * @author Li Shu
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct GoalPost
 * Description of a perceived goal post
 */
STREAMABLE(GoalPost,
{
  ENUM(Direction,
  {,
    IS_UNKNOWN,
    IS_LEFT,
    IS_RIGHT,
  }),

  (Direction)(IS_UNKNOWN) direction, 	/**< Direction of this post */
  (Vector2i) positionInImage, 		/**< The position of the goal post in the current image */
  (Vector2f) positionOnField, 		/**< The position of the goal post relative to the robot*/
  (Vector2i) basePointInImage,		/**< Base point of a goal in image */
  (Vector2i) topPointInImage,		/**< Top point of a goal in image (if top point found) */
  (bool) topPointFound,				/**< Is top point of a goal post found? */
  (float) possibility,				/**< The possibility te be a real goal post */
});


/**
 * @struct GoalPercept
 * Set of perceived goal posts
 */
STREAMABLE(GoalDetect2017,
{
  /** Draws the perceived goal posts*/
  void draw() const,

  (std::vector<GoalPost>) goalPosts, 			/**<Is empty if no goal posts where seen this frame */
  (unsigned)(0) timeWhenGoalPostLastSeen, 		/**< Time when a goal post was seen. */
  (unsigned)(0) timeWhenCompleteGoalLastSeen, 	/**< Time when complete goal was seen. */
});
