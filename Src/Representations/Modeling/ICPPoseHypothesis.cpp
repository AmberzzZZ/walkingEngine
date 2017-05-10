/**
 * @file ICPPoseHypothesis.cpp
 *
 * A new robot pose, based on recently observed field features.
 * This pose is meant to be used for sensor resetting inside the SelfLocalization
 *
 * @author Zeng Zhiying
 */

#include "ICPPoseHypothesis.h"
#include "Tools/Debugging/DebugDrawings.h"

void ICPPoseHypothesis::draw() const
{
  DEBUG_DRAWING("representation:ICPPoseHypothesis", "drawingOnField")
  {
    if(isValid)
    {
      ColorRGBA col = isInOwnHalf ? ColorRGBA(180, 180, 255) : ColorRGBA(255, 0, 0);
      DRAW_ROBOT_POSE("representation:ICPPoseHypothesis", pose, ColorRGBA(0,0,0));
      CIRCLE("representation:ICPPoseHypothesis", pose.translation.x(), pose.translation.y(),
             500, 40, Drawings::solidPen, col, Drawings::noBrush, col);
      DRAWTEXT("representation:ICPPoseHypothesis", pose.translation.x(), pose.translation.y()+700, 200,
                     ColorRGBA(0,0,0), "ICP");
    }
  }
}
