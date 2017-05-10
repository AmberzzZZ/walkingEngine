#include "../LibraryBase.h"

namespace Behavior2015
{
#include "../Libraries/LibWalkToTarget_SiQiang.h"
#include "../Libraries/LibBall.h"
#include "../Libraries/LibRobot.h"

LibWalkToTarget::LibWalkToTarget()
  {
	 target = Vector2f(-100,0.f);//field
	 goal = Vector2f(2100.f,0.f);//field
  }

  void LibWalkToTarget::preProcess()
  {

  }
  void LibWalkToTarget::postProcess()
  {
//	  //if ball was not seen, make parameters invalid
//	  if(!ball.wasSeen())
//	  {
//		  rhoValue = 9000.f;
//		  psiValue = 9000.f;
//		  phiValue = (Angle)180_deg;
//		  gammaValue = (Angle)180_deg;
//		  betaValue = (Angle)180_deg;
//		  alphaValue = (Angle)180_deg;
//	  }
  }

  void LibWalkToTarget::GetAllParameterValue(float &kx,float &ky,float &kthetagamma, float &kthetaphivalue,Vector2f &target_1,Vector2f &assignTarget_1)
  {
	  target = target_1;
//	  assignTarget = assignTarget_1;
	  goal = assignTarget_1;
	  GetParameter();
	  kx = GetKxValue();
	  ky = GetKyValue();
	  kthetagamma = GetKthetagammaValue();
	  kthetaphivalue = GetKthetaphiValue();
  }
  void LibWalkToTarget::GetParameter()
    {
	  //like alignFuzzy, just use target as ball.global and use assignTarget as goal
  	  Vector2f goalToRobot = theRobotPose.inverse() * goal;
  	  Vector2f targetToRobot = theRobotPose.inverse() * target;

   	  rhoValue = targetToRobot.norm();
   	  psiValue = (targetToRobot-goalToRobot).norm();
   	  alphaValue =  goalToRobot.angle();
   	  gammaValue = targetToRobot.angle();
   	  if(fabs((sqr(goalToRobot.norm())+sqr(psiValue)-sqr(rhoValue))/(2.f*goalToRobot.norm()*psiValue)) > 1)
   	  {
   		  if(target.x() >= robot.x)
   		  {
   			  betaValue = 0.f;
   		  }
   		  else
   		  {
   			  betaValue = (Angle) 180_deg;
   		  }

   	  }
   	  else if(goalToRobot.norm()*psiValue == 0)
   	  {
   		  betaValue = 0.f;
   	  }
   	  else
   	  {
   		 betaValue = acos((sqr(goalToRobot.norm())+sqr(psiValue)-sqr(rhoValue))/(2.f*goalToRobot.norm()*psiValue));

   	  }
   	  phiValue = fabs(betaValue + fabs(alphaValue - gammaValue));

//   	 Geometry::Line BallToGoal = Geometry::Line(ball.global,goal-ball.global);
   	 Geometry::Line LineToRobot = Geometry::Line(robot.pose.translation,goal-target);
   	 Vector2f intersection;
   	 Geometry::getIntersectionOfLines(LineToRobot, Geometry::Line(Vector2f(0, 0), Vector2f(1, 0)),intersection);
   	 if(target.y() - goal.y() < 0.1f && robot.y - target.y()>0.f)
   	 {
   		 phiValue = -phiValue;
   	 }
   	 else if(target.y() - goal.y()<0.f && intersection.x()<goal.x())
   	 {
   		 phiValue = -phiValue;
   	 }
   	 else if(target.y() - goal.y()>0.f && intersection.x()>goal.x())
   	 {
   		 phiValue = -phiValue;
   	 }
   }
  float LibWalkToTarget::GetKxValue()
  {
	  int x,y;
//	  goal = target;
	  x = GetGammaSemanticValue();
	  y = GetPhiSemanticValue();
	  return kxMembership[x+3][y];
  }
  float LibWalkToTarget::GetKyValue()
  {
	  int x,y;
//	  goal = target;
	  x = GetPhiSemanticValue();
	  y = GetRhoSemanticValue();
	  return kyMembership[x][y];
  }
  float LibWalkToTarget::GetKthetagammaValue()
  {
	  int x;
//	  goal = target;
	  x = GetRhoSemanticValue();
	  return kthetagammaMembership[x];
  }
  float LibWalkToTarget::GetKthetaphiValue()
  {
	  int x;
//	  goal = target;
	  x = GetRhoSemanticValue();
	  return kthetaphiMembership[x];
  }
  int LibWalkToTarget::GetGammaSemanticValue()
  {
	  if(gammaValue > 70_deg) return 3;
	  else if(gammaValue > 40_deg) return 2;
	  else if(gammaValue > 15_deg) return 1;
	  else if(gammaValue > -15_deg) return 0;
	  else if(gammaValue > -40_deg) return -1;
	  else if(gammaValue > -70_deg) return -2;
	  else return -3;
  }
  int LibWalkToTarget::GetPhiSemanticValue()
  {
	  if(fabs(phiValue) > 90_deg) return 2;
	  else if(fabs(phiValue) > 45_deg) return 1;
	  else  return 0;
  }
  int LibWalkToTarget::GetRhoSemanticValue()
  {
	  if(rhoValue > 3000.f) return 3;
	  else if(rhoValue > 1000.f) return 2;
	  else if(rhoValue > 500.f) return 1;
	  else  return 0;
  }
//  int LibAlignFuzzy::GetKxSemanticValue()
//  {
//	  if(ball.position)
//  }
}
