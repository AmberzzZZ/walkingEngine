/**
* @file GoalPercept2017.cpp
*
* representation of a seen goal
*
* @author Li Shu
*/

#include "GoalPercept2017.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Debugging/DebugDrawings.h"

void GoalPercept2017::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:GoalPercept2017:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:GoalPercept2017:Field", "drawingOnField");

  for (int i=0; i<numberOfGoalPosts; i++)
  {
      QUADRANGLE("representation:GoalPercept2017:image",
        (goalPosts[i].topInImage.x() - goalPosts[i].topWidth / 2),
        (goalPosts[i].topInImage.y()),
        (goalPosts[i].topInImage.x() + goalPosts[i].topWidth / 2),
        (goalPosts[i].topInImage.y()),
        (goalPosts[i].bottomInImage.x() + goalPosts[i].bottomWidth / 2),
        (goalPosts[i].bottomInImage.y()),
        (goalPosts[i].bottomInImage.x() - goalPosts[i].bottomWidth / 2),
        (goalPosts[i].bottomInImage.y()),
        4, Drawings::solidPen, ColorRGBA::yellow);

//    Vector2i center = ( goalPosts[i].topInImage
//                          + goalPosts[i].bottomInImage ) / 2;
		LINE("representation:GoalPercept2017:image",
			goalPosts[i].topInImage.x(),
			goalPosts[i].topInImage.y(),
			goalPosts[i].topInImage.x(),
			0,
			4,
			Drawings::dottedPen,
			ColorRGBA::yellow);

//      CIRCLE("representation:GoalPercept2017:Image", center.x(), center.y(),
//        5, 2, Drawings::solidPen, ColorRGBA::gray, Drawings::noBrush, ColorRGBA::gray);
//    int factor = -1;
    if (goalPosts[i].goalPostSide == GoalPost::leftPost)
    {
		DRAWTEXT("representation:GoalPercept2017:image",
				goalPosts[i].topInImage.x()+5,
				30,
				30,
				ColorRGBA::yellow,
				"L");
    }
	else if (goalPosts[i].goalPostSide == GoalPost::rightPost)
	{
		DRAWTEXT("representation:GoalPercept2017:image",
				goalPosts[i].topInImage.x()-30,
				30,
				30,
				ColorRGBA::yellow,
				"R");
	}
    if (goalPosts[i].goalPostSide != GoalPost::unknownPost)
    {
//      if (goalPosts[i].fromUpper)
//        ARROW("representation:GoalPercept2017:Image", center.x(), center.y(), center.x() + factor * 30, center.y(),
//          2, Drawings::solidPen, ColorRGBA::white);
//      else
//        ARROW("representation:GoalPercept2017:Image",center.x(),center.y(),center.x()+factor*30,center.y(),
//          2,Drawings::solidPen, ColorRGBA::white);
    }


//    LINE("representation:GoalPercept2017:Field",0,0,goalPosts[i].locationOnField.x(), goalPosts[i].locationOnField.y(),
//      10, (goalPosts[i].bottomFound ? Drawings::dottedPen : Drawings::solidPen), ColorRGBA::yellow);
    CIRCLE("representation:GoalPercept2017:Field", goalPosts[i].locationOnField.x(), goalPosts[i].locationOnField.y(),
      60, 10, Drawings::solidPen, ColorRGBA::white, Drawings::solidBrush, ColorRGBA::yellow);
    
//    if (goalPosts[i].goalPostSide != GoalPost::unknownPost)
//    {
//      Vector2f dir(static_cast<float>(goalPosts[i].locationOnField.y()), static_cast<float>(-goalPosts[i].locationOnField.x()));
//      ColorRGBA arrowColor = ColorRGBA::gray;
//      ARROW("representation:GoalPercept2017:Field",
//            goalPosts[i].locationOnField.x(),
//            goalPosts[i].locationOnField.y(),
//            goalPosts[i].locationOnField.x() + dir.x(),
//            goalPosts[i].locationOnField.y() + dir.y(),
//            20,Drawings::solidPen, arrowColor);
//    }
  }

}
