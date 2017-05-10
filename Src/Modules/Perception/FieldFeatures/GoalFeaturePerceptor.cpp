#include "GoalFeaturePerceptor.h"
#include "Tools/Math/Transformation.h"
#include <vector>
#include <iostream>


void GoalFeaturePerceptor::update(GoalFeature& goalFeature)
{
  goalFeature.clear();
  goalFeature.isValid = false;
  if(theGoalPercept2017.goalPosts.size() == 2)
  {
	  ;
  }
  else
	  return;
  float minDis = 10000;
  int minIdx1 = 0;
  int minIdx2 = 1;
//  std::cout<<"goalsize--->"<<theGoalPercept2017.goalPosts.size()<<std::endl;
//  for(int i = 0; i < theGoalPercept2017.goalPosts.size() - 1; i++)
//  {
//	  Vector2f Post1 = theGoalPercept2017.goalPosts[i].locationOnField;
//	  for(int j = i + 1; j < theGoalPercept2017.goalPosts.size(); j++)
//	  {
//		  Vector2f Post2 = theGoalPercept2017.goalPosts[j].locationOnField;
//		  float dis = (Post2 - Post1).norm();
//		  if(fabs(dis - 1600.f) < minDis)
//		  {
			  minDis = fabs((theGoalPercept2017.goalPosts[minIdx1].locationOnField - theGoalPercept2017.goalPosts[minIdx2].locationOnField).norm() - 1500.f);
//			  minIdx1 = i;
//			  minIdx2 = j;
//		  }
//	  }
//  }
//  if(minDis < 300)
//	  return;
  std::cout<<"ok2>"<<std::endl;
  Vector2f GoalCenterPos = (theGoalPercept2017.goalPosts[minIdx1].locationOnField + theGoalPercept2017.goalPosts[minIdx2].locationOnField) / 2;
  const Geometry::Line geomLine(theGoalPercept2017.goalPosts[minIdx1].locationOnField, theGoalPercept2017.goalPosts[minIdx2].locationOnField - theGoalPercept2017.goalPosts[minIdx1].locationOnField);
  const float rawDistanceToLine = Geometry::getDistanceToLine(geomLine, Vector2f(0,0));
//  const Vector2f offset = geomLine.direction.normalized(100).rotate(-pi_2);
  const Vector2f offset = geomLine.direction.normalized(100).rotate(rawDistanceToLine > 0 ? pi_2 : -pi_2);
  Vector2f dir= (theGoalPercept2017.goalPosts[minIdx2].locationOnField - theGoalPercept2017.goalPosts[minIdx1].locationOnField);
  if(dir.y() < 0)
	  dir = -dir;
  dir.rotate(pi_2);
  const Pose2f goalPose(offset.angle(), GoalCenterPos);
  std::cout<<"ok3>"<<std::endl;
  goalFeature = goalPose;

//  std::cout<<"goalFeature--->"<<goalPose.translation.x()<<","<<goalPose.translation.y()<<","<<goalPose.rotation<<std::endl;

  goalFeature.isValid = true;
}


MAKE_MODULE(GoalFeaturePerceptor, perception)
