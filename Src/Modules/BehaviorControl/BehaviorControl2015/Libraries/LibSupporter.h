/**
 * @class LibSupporter
 * @author Li Shu <tongjilishu@163.com>
 * @date 13/04/16
 * @file LibSupporter.h
 * @brief guess it ! ^_^
 */

#include "LibIntentions.h"

class LibSupporter : public LibraryBase
{
 public:
  LibSupporter();

  void preProcess() override;

  void postProcess() override;

  float yOnGoal;
  
  Vector2f ballPosition;
  Vector2f ballPositionRelative;
  Vector2f ballSpeedRelative;
  float yIfBallPassOwnAxis;

  const float dangerLine = -1500.f;
  const float robotSpeedX = 100.f;
  
  bool wrongLacation;
  bool useTeamBall;
  bool lastIsLeft = false;
  
  SupporterIntention currentIntention;
  SupporterIntention preIntention;
  
  Geometry::Line defenderLine = Geometry::Line(Vector2f(-3200, 0),
                                               Vector2f(0, 1));
  Geometry::Line supporterAt2400 = Geometry::Line(Vector2f(-2400, 0),
                                                 Vector2f(0, 1));
  Geometry::Line supporterAt1600 = Geometry::Line(Vector2f(-1600, 0),
                                                 Vector2f(0, 1));
  Geometry::Line supporterAt0800 = Geometry::Line(Vector2f(-800, 0),
                                                 Vector2f(0, 1));
  Geometry::Line supporterAt0000 = Geometry::Line(Vector2f(0, 0),
                                                 Vector2f(0, 1));

  Vector2f defenderPositionWhenKeeperPenalized();  //站中间
  Vector2f defenderPositionWhenKeeperNotPenalized();  //站左边
  Vector2f supporterPositionWhenKeeperNotPenalized();  //站右边

  Vector2f supporterStandPosition();             /**< supporter position when defence or offence */
  float yPoseCoorperateWithStrikerAndStabber();  /**< y position, calculated by the state and position Striker and Stabber */
  
  SupporterIntention NextIntentionDeterminedByCurrent(SupporterIntention it, Vector2f ballSpeedRelative, Vector2f positionAbsolute, Vector2f endPosition, Vector2f endPositionRelative);
  
  Vector2f SupportPositionDeterminedByIntention(SupporterIntention it);
  
  Vector2f getBallSpeedRelative();
  float getYIfBallPassOwnAxis();
  Vector2f getBallEndPoint();
  
  Vector2f getBallEndPointRelative();
  
  Vector2f ballPositionFromKeeper();             /**< ball position by keeper */
  Vector2f ballPositionFromDefender();           /**< ball position by defender */
  
  Vector2f getBallRelativePosition();            /**< ball position combined with whole team */
  Vector2f getBallAbsolutePosition();            /**< ball position combined with whole team */
  Vector2f getSelfLocation();                    /**< supporter pisition by its SelfLocator and team communication */

  int ballGoAway = -1;
  float lastBallX = 0;
  float lastStandY = 10000;

  Vector2f defenderPositionCoorperateWithKeeperAndSupporter(bool left);
  float yIfBallCanPassGroundLine();  //球朝自家球门运动时，估测球运动到自家球门时的y值
  bool ifStopBallNeeded();  //球的运动轨迹会进自家球门时为true
  bool toStopBall();  //ifStopBallNeeded()为true，且球离自己的距离足够近时为true
  Pose2f defenderDefencePosition();  //相对位置
  Pose2f supporterDefencePosition();  //相对位置
  bool ifSpreadNeeded();  //toStopBall()为ture，且球会从自己身旁经过时为true
  bool ifInterceptNeeded();
  bool robotAndBallCloseEnough();  //球离自己足够近时为true
  Vector2f globalSpeed(Vector2f relativeSpeed);

  Vector2f supporterStandPos();
  float supporterStandPosY(float standPosX);

  Pose2f supporterPosition();

};
