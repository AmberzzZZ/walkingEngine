/**
 * @file BallPercept.cpp
 *
 * Very simple representation of a seen ball
 *
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 * @autor Jesse
 */

#include "BallPercept2017.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void BallPercept2017::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:BallPercept2017:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:BallPercept2017:field", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:BallPercept2017", "robot");
  TRANSLATE3D("representation:BallPercept2017", 0, 0, -230);
  if(status == seen)
  {
    CIRCLE("representation:BallPercept2017:image", positionInImage.x(), positionInImage.y(), radiusInImage, 2, // pen width
           Drawings::solidPen,  ColorRGBA::black, Drawings::solidBrush, ColorRGBA(255, 128, 64, 200));
    CIRCLE("representation:BallPercept2017:field", positionOnField.x(), positionOnField.y(), radiusOnField, 0, // pen width
           Drawings::solidPen, ColorRGBA::orange, Drawings::noBrush, ColorRGBA::orange);
    SPHERE3D("representation:BallPercept2017", positionOnField.x(), positionOnField.y(), radiusOnField, radiusOnField, ColorRGBA::orange);
  }
  else if(status == guessed)
    CIRCLE("representation:BallPercept2017:image", positionInImage.x(), positionInImage.y(), radiusInImage, 1, // pen width
           Drawings::solidPen,  ColorRGBA::black, Drawings::solidBrush, ColorRGBA(64, 128, 255, 90));
}
