/*
 * DynamicDribble.h
 *
 *  Created on: Jun 18, 2016
 *      Author: yongqi
 */

Vector2f nextTarget = Vector2f(0.f, 0.f);
//Vector2f lastTarget = Vector2f(0.f, 0.f);
option(DynamicDribble, (float) x, (float) y)
{

  DEBUG_DRAWING("representation:Role", "drawingOnField")
  {
    CIRCLE("representation:Role",
           nextTarget.x(), nextTarget.y(), 45, 0, // pen width
           Drawings::solidPen, ColorRGBA::blue,
           Drawings::solidBrush, ColorRGBA::blue);
  }
//  std::cout<<"next step, x: "<<nextTarget.x()<<" y: "<<nextTarget.y()<<std::endl;

  common_transition
  {
    if(fabs(ball.global.x() - x) < 50 && fabs(ball.global.y() - y) < 50)
      goto finish;
  }

  initial_state(dribble2nextPoint)
  {
    transition
    {
      if(state_time % 3000 == 0)
      {
//        lastTarget = nextTarget;
        nextTarget = walk.findPath(Vector2f(x, y));
//        nextTarget = walk.findNextTarget(Vector2f(x, y));
//        if((nextTarget - lastTarget).norm() > 1500)
//        {
//          nextTarget = lastTarget;
//          std::cout<< "triger\n";
//        }
      }
    }
    action
    {
      Dribble2(nextTarget.x(), nextTarget.y());
    }
  }

  target_state(finish)
  {
    transition
    {
      if(fabs(ball.global.x() - x) > 50 || fabs(ball.global.y() - y) > 50)
        goto dribble2nextPoint;
    }
    action
    {
      WalkAtSpeedPercentage(Pose2f(0.f, 0.f, 0.f));
    }
  }
}
