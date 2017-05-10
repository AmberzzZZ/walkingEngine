#include "../LibraryBase.h"
 
namespace Behavior2015
{
#include "../Libraries/LibAlignFuzzy.h"
#include "../Libraries/LibBall.h"
#include "../Libraries/LibRobot.h"

  LibAlignFuzzy::LibAlignFuzzy()
  {
	goal = Vector2f(4100.f,0.f);//field
	useLeftFoot = true;
  }

  void LibAlignFuzzy::preProcess()
  {

  }
  void LibAlignFuzzy::postProcess()
  {
	  //if ball was not seen, make parameters invalid
	  if(!ball.wasSeen() && !ball.isTeamPositionValid)
	  {
		  rhoValue = 9000.f;
		  psiValue = 9000.f;
		  phiValue = (Angle)180_deg;
		  gammaValue = (Angle)180_deg;
		  betaValue = (Angle)180_deg;
		  alphaValue = (Angle)180_deg;
	  }
  }

  void LibAlignFuzzy::GetAllParameterValue(float &kx,float &ky,float &kthetagamma, float &kthetaphivalue,Vector2f &target,bool &useLeftOrRight)
  {
	  useLeftFoot = useLeftOrRight;
	  GetParameter(target,useLeftOrRight);
	  kx = GetKxValue(target);
	  ky = GetKyValue(target);
	  kthetagamma = GetKthetagammaValue(target);
	  kthetaphivalue = GetKthetaphiValue(target);
  }
  void LibAlignFuzzy::GetParameter(Vector2f &target,bool &useLeftOrRight)
    {
  	  //judge use left or right foot kick
  	 goal = target;
  	 Vector2f goalToRobot = theRobotPose.inverse() * Vector2f(goal.x(), goal.y());
  	 Vector2f virtualGoalToRobot,virtualBallToRobot;
  	 float modifyDistance = 40.f;
  	 float backwardsDistance = 80.f;
  	 float sigmaValue;
  	 sigmaValue = (ball.positionRobot - goalToRobot ).angle();
  //	 Vector2f omega;
  //	 omega.x() =  (ball.position - theRobotPose.inverse() * Vector2f(goal.x(), goal.y()) ).x() / ( ball.position - theRobotPose.inverse() * Vector2f(goal.x(), goal.y()) ).norm();
  //	 omega.y() =  (ball.position - theRobotPose.inverse() * Vector2f(goal.x(), goal.y()) ).y() / ( ball.position - theRobotPose.inverse() * Vector2f(goal.x(), goal.y()) ).norm();
  	 Vector2f sideward;
  	 if(useLeftOrRight)
  	 {
  		 sideward = Vector2f( modifyDistance*cos(sigmaValue + (Angle)90_deg) , modifyDistance*sin(sigmaValue + (Angle)90_deg));
  	 }
  	 else
  	 {
  		 sideward = Vector2f( modifyDistance*cos(sigmaValue - (Angle)90_deg) , modifyDistance*sin(sigmaValue - (Angle)90_deg));
  	 }
  	 sideward = sideward + Vector2f(backwardsDistance*cos(sigmaValue),backwardsDistance*sin(sigmaValue));
//  	 virtualGoalToRobot = goalToRobot + sideward;
	   virtualGoalToRobot = goalToRobot;
  	 virtualBallToRobot = ball.positionRobot + sideward;
  	  rhoValue = virtualBallToRobot.norm();
  	  psiValue = (virtualBallToRobot-virtualGoalToRobot).norm();
  	  alphaValue =  virtualGoalToRobot.angle();
  	  gammaValue = virtualBallToRobot.angle();
//   	  rhoValue = ball.position.norm();
//   	  psiValue = (ball.position-goalToRobot).norm();
//   	  alphaValue =  goalToRobot.angle();
//   	  gammaValue = ball.position.angle();
   	  if(fabs((sqr(goalToRobot.norm())+sqr(psiValue)-sqr(rhoValue))/(2.f*goalToRobot.norm()*psiValue)) > 1)
   	  {
   		  if(ball.global.x() >= robot.x)
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
   		  betaValue = acos((sqr(virtualGoalToRobot.norm())+sqr(psiValue)-sqr(rhoValue))/(2.f*virtualGoalToRobot.norm()*psiValue));
//   		 betaValue = acos((sqr(goalToRobot.norm())+sqr(psiValue)-sqr(rhoValue))/(2.f*goalToRobot.norm()*psiValue));

   	  }
   	  phiValue = fabs(betaValue + fabs(alphaValue - gammaValue));

  	 Geometry::Line BallToGoal = Geometry::Line(ball.global,goal-ball.global);
   	 Geometry::Line LineToRobot = Geometry::Line(robot.pose.translation,goal-ball.global);
   	 Vector2f intersection;
     Vector2f intersection2;
   	 Geometry::getIntersectionOfLines(LineToRobot, Geometry::Line(Vector2f(0, 0), Vector2f(1, 0)),intersection);
     Geometry::getIntersectionOfLines(BallToGoal, Geometry::Line(Vector2f(0, 0), Vector2f(1, 0)),intersection2);
   	 if(ball.global.y() - goal.y() < 0.1f && robot.y - ball.global.y()>0.f)
   	 {
   		 phiValue = -phiValue;
   	 }
   	 else if(ball.global.y() - goal.y()<0.f && intersection.x()<intersection2.x())
   	 {
   		 phiValue = -phiValue;
   	 }
   	 else if(ball.global.y() - goal.y()>0.f && intersection.x()>intersection2.x())
   	 {
   		 phiValue = -phiValue;
   	 }
    }
  float LibAlignFuzzy::GetKxValue(Vector2f &target)
  {
	  int x,y;
//	  goal = target;
	  x = GetGammaSemanticValue();
	  y = GetPhiSemanticValue();
	  return kxMembership[x+3][y];
  }
  float LibAlignFuzzy::GetKyValue(Vector2f &target)
  {
	  int x,y;
//	  goal = target;
	  x = GetPhiSemanticValue();
	  y = GetRhoSemanticValue();
	  return kyMembership[x][y];
  }
  float LibAlignFuzzy::GetKthetagammaValue(Vector2f &target)
  {
	  int x;
//	  goal = target;
	  x = GetRhoSemanticValue();
	  return kthetagammaMembership[x];
  }
  float LibAlignFuzzy::GetKthetaphiValue(Vector2f &target)
  {
	  int x;
//	  goal = target;
	  x = GetRhoSemanticValue();
	  return kthetaphiMembership[x];
  }
  int LibAlignFuzzy::GetGammaSemanticValue()
  {
	  if(gammaValue > 70_deg) return 3;
	  else if(gammaValue > 40_deg) return 2;
	  else if(gammaValue > 15_deg) return 1;
	  else if(gammaValue > -15_deg) return 0;
	  else if(gammaValue > -40_deg) return -1;
	  else if(gammaValue > -70_deg) return -2;
	  else return -3;
  }
  int LibAlignFuzzy::GetPhiSemanticValue()
  {
	  if(fabs(phiValue) > 90_deg) return 2;
	  else if(fabs(phiValue) > 45_deg) return 1;
	  else  return 0;
  }
  int LibAlignFuzzy::GetRhoSemanticValue()
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
