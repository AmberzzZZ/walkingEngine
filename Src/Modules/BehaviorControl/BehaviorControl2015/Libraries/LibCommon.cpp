/**
 * @file LibCommon.cpp
 */

#include "../LibraryBase.h"

namespace Behavior2015
{
#include "LibBall.h"
#include "LibCommon.h"
#include "LibRobot.h"

  LibCommon::LibCommon()
  {
  }

  void LibCommon::preProcess()
  {
//      std::cout<<"isKickOff = "<<isKickOff<<std::endl;
//    static int cnt = 0;
//    cnt = (cnt + 1) % 10;
//    if(cnt == 0)
//    {
//      lastGameState = gameState;
//      gameState = theGameInfo.state;
//    }
	  gameState = theGameInfo.state;
//	  std::cout<<"gameState---->"<<gameState<<std::endl;
//	  std::cout<<"lastGameState---->"<<lastGameState<<std::endl;
//	  std::cout<<"iskickoff----->"<<isKickOff<<std::endl;
    if(lastGameState == STATE_SET && gameState == STATE_PLAYING)
    {
      isKickOff = true;
      isKickOffLimit = true;
      kickOffTime = 0;
    }
    if(isKickOff)
    {
      ++kickOffTime;
    }
//    std::cout<<"kickOffTime = "<<kickOffTime<<std::endl;
    if(kickOffTime > 60 * 10 )
    {
      isKickOff = false;
    }

    if ((ball.wasSeen() || ball.isTeamPositionValid ) && ball.positionField.norm() > 650)
    {
//    	std::cout<<"ball.positionField.norm(-------->"<<ball.positionField.norm()<<std::endl;
        isKickOff = false;
    }

    if(isKickOffLimit && ball.positionField.norm() > 750)
    {
      isKickOffLimit = false;
//      std::cout<<"no kickOffLimit"<<std::endl;
    }
    lastGameState = gameState;
//    std::cout<<"isKickOff = "<<isKickOff<<std::endl;
//    std::cout<<"ball.global = "<<ball.global.x()<<", "<<ball.global.y()<<std::endl;
  }

  void LibCommon::postProcess()
  {

  }

  bool LibCommon::between(float value, float min, float max)
  {
    return value >= min && value <= max;
  }

  float LibCommon::fromDegrees(float degrees)
  {
    return degrees / 180 * (float) pi;
  }

  float LibCommon::fromRad(float rad)
  {
    return rad / (float) pi * 180;
  }

  float LibCommon::angleToPose(Pose2f pose)
  {
    return (float) atan2(pose.translation.y() - robot.y,
                         pose.translation.x() - robot.x);
  }

  float LibCommon::radToPose(Pose2f pose)
  {
    float angle = (float) atan2(pose.translation.y() - robot.y,
                                pose.translation.x() - robot.x);
    return fromDegrees(angle);
  }

  float LibCommon::getDistanceSqr(const Vector2f &vec1, const Vector2f &vec2)
  {
    return (vec1.x() - vec2.x()) * (vec1.x() - vec2.x()) +
        (vec1.y() - vec2.y()) * (vec1.y() - vec2.y());
  }

  void LibCommon::stepClipped(float& forward, float& left, Angle& turn)
  {
	  Angle turnClip = 0.8f;
	    if(fabs(turn) > 40_deg)
		{
			forward = 0;
			left = 0;
		}
		else if(fabs(forward) < 100.f && fabs(left) < 80.f)
		{
			turnClip = 0.3f;
		}
		else if(abs(left) > fabs(forward) && fabs(left) > 100.f)
		{
			forward = 0;
		}
		else if(forward > 200)
		{
			left = 0;
			turn = 0;
		}
		if(forward > 300.f)
		{
			forward = 300.f;
		}
		else if(forward < -300.f)
		{
			forward = -300.f;
		}
		if(left > 200.f)
		{
			left = 200.f;
		}
		else if(left < -200.f)
		{
			left = -200.f;
		}
		if(turn > turnClip)
		{
			turn = turnClip;
		}
		else if(turn < -turnClip)
		{
			turn = -turnClip;
		}
		if(fabs(forward) == 0 and fabs(left) == 0 and fabs(turn) ==0)
			forward = 3;
  }

}
