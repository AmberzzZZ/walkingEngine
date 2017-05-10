/**
* @file LibBall.cpp
*/

#include "../LibraryBase.h"
#include <iostream>

namespace Behavior2015
{
  #include "LibWalk.h"
  #include "LibObstacle.h"
  #include "LibTeammate.h"

  
  LibWalk::LibWalk():toAvoid(false),avoidToLeft(false),thePathFinder(theFieldDimensions,theRobotInfo,theTeamPlayersModel)
  {
  }
  
  void LibWalk::preProcess()
  {
  }

  void LibWalk::postProcess()
  {
  }
//
//  Vector2f LibWalk::findPath(Vector2f startLocation,Vector2f targetLocation)
//  {
//
//
//	  PathFinder thePathFinder(obstacle.getAllObstaclesPosition(),teammate.getAllTeammatesPose(),theFieldDimensions,theRobotInfo);
//	  thePathFinder.findPath(startLocation,targetLocation);
//	  for(std::vector<Vector2f>::const_iterator i = thePathFinder.path.begin(), end = thePathFinder.path.end(); i != end; ++i)
//	  {
//		  DOT("representation:AvoidPath:path",i->x(), i->y(), ColorRGBA::black, ColorRGBA::black)	;
//	  }
//	  return *thePathFinder.path.begin();
//  }


  Vector2f LibWalk::findPath(Vector2f targetLocation)
  {
	  thePathFinder.findPath(theRobotPose.translation,targetLocation);
	  for(std::vector<Vector2f>::const_iterator i = thePathFinder.path.begin(), end = thePathFinder.path.end() - 1; i != end; ++i)
	  {
		  LINE("representation:AvoidPath:path", i->x()+900, i->y(), (i+1)->x()+900, (i+1)->y(), 30, Drawings::PenStyle::solidPen, ColorRGBA::blue);
	  }
	  return *(thePathFinder.path.begin()+3);
  }

//  Vector2f LibWalk::findPath()
//  {
//	  thePathFinder.findPath(theRobotPose.translation,theCombinedWorldModel.ballState.position);
//	  return *thePathFinder.path.begin();
//  }
  
  bool LibWalk::isContactLeft()
  {
	  return theArmContactModel.contactLeft;
  }

  bool LibWalk::isContactRight()
  {
	  return theArmContactModel.contactRight;
  }

  Vector2f LibWalk::findNextTarget(Vector2f finalTarget)
  {
    Vector2f robot = theRobotPose.translation;
    Vector2f globalObs = Transformation::robotToField(theRobotPose, obstacle.center);
    Vector2f nextTarget;
    float distance = (robot - finalTarget).norm();
    // step 1
    if (distance < 1000)
      nextTarget = finalTarget;
    else
      nextTarget = robot + 1000 / distance * (finalTarget - robot);
    // step 2
    nextTarget = needAvoidObstcale(robot, nextTarget, globalObs);
//    nextTarget = needAvoidObstcale(robot, nextTarget, Vector2f(-1000, 0));
    return nextTarget;
  }

  Vector2f LibWalk::needAvoidObstcale(Vector2f beg, Vector2f end, Vector2f obs)
  {
    const float eplison = 1.f;
    const float a = 500.f / 2.f;
    const float b = (beg - end).norm() / 2.f;
    float k = (end.y() - beg.y()) / (end.x() - beg.x());
    float d1, d2;
    Vector2f cross, res;

    if (fabs(end.y() - beg.y()) < eplison)
    {
      // y - beg.y() = 0; x - (beg.x() + end.x()) / 2 = 0;
      d1 = static_cast<float>(fabs(obs.y() - beg.y()));
      d2 = static_cast<float>(fabs(obs.x() - (beg.x() + end.x()) / 2));
      cross.x() = obs.x();
      cross.y() = beg.y();
    }
    else if (fabs(end.x() - beg.x()) < eplison)
    {
      // x - beg.x() = 0; y - (beg.y() + end.y()) / 2 = 0;
      d1 = static_cast<float>(fabs(obs.x() - beg.x()));
      d2 = static_cast<float>(fabs(obs.y() - (beg.y() + end.y()) / 2));
      cross.x() = beg.x();
      cross.y() = obs.y();
    }
    else
    {
      float factor = 1.f / static_cast<float>(sqrt(1 + k*k));
      Vector2f mid = (beg + end) / 2.f;
      d1 = factor * static_cast<float>(fabs(k*obs.x() - obs.y() + beg.y() - k*beg.x()));
      d2 = factor * static_cast<float>(fabs(obs.x() + k*obs.y() - k*mid.y() - mid.x()));
      cross.x() = (k*k*beg.x() + obs.x() + k*obs.y() - k*beg.y()) / (k*k + 1);
      cross.y() = (k*k*obs.y() + beg.y() + k*obs.x() - k*beg.x()) / (k*k + 1);
    }
//    std::cout<<"obs:"<<obs.x()<<" "<<obs.y()<<std::endl;
//    std::cout<<"d1:"<<d1<<" d2:"<<d2<<" b:"<<b<<std::endl;
    float obsDistance = (obs - beg).norm();
    if (d1 < a && d2 < b && obsDistance > 150.f && obsDistance < 1000.f)
    {
//      std::cout<<"obsDistance:"<<obsDistance<<std::endl;
      float factor = (1.f/obsDistance - 0.001f) / 0.004f * 200.f + 200.f;
      cross -= obs;
      cross.x() /= fabs(cross.x());
      cross.y() /= fabs(cross.y());
      res = end + factor * cross;
      res.y() *= 3.f;
//      std::cout<<factor<<std::endl;
//      std::cout<<"res:"<<res.x()<<" "<<res.y()<<std::endl;
    }
    else
    {
      res = end;
    }
    return res;
  }

}
