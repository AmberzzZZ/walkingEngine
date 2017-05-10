 /**
  * @class LibBall
  * @author Li_Shu <a href="mailto:tongjilishu@163.com">Li Shu</a>
  * @date 11/04/2016
  * @file LibBall.h
  * @brief 
  */

/**  
 * @TODO
 
  * Accurately calculate timeWhenAxisReached
  * Accurately calculate yPosWhenBallReachesOwnYAxis, maybe by using ball.estimate information.
  * 
  * 
  * */

#pragma once

class LibBall : public LibraryBase
{
 public:
  LibBall();

  void preProcess() override;
  void postProcess() override;

  Vector2f position;
  Vector2f endPosition;
  Vector2f endPositionGlobal;
  float &x = position.x();      /**< x-coordinate of ball position relative to robot in mm(+ is in front) */
  float &y = position.y();      /**< y-coordinate of ball position relative to robot in mm(+ is left) */
  float distance;               /**< distance from robot to ball in mm */
  float angleDeg;               /**< direction to ball relative to robot in degrees(0 is front,90 is left) */
  float angleRad;               /**< direction to ball relative to robot in rad(0 is front,pi/4 is left) */
  float timeWhenAxisReached;    /**< time when the ball reach own axis in ms */
  Vector2f global;              /**< ball position on the field coordinate, seen by the robot itself */
  Vector2f speedRelative;       /**< speed of the Ball relative to the Robot in "mm/s" */
  /* ball position on the field global(if one can see the ball, it is
   * ball.global or it is the ball.teamPosition and I have take ball.notSeenTime
   * and ball.isTeamPositionValid into account). if who want to use it, he must
   * call getSuitablePOsition(). */
  Vector2f suitable;            /**< ball position combined own observation and team ball information */

  bool isTeamPositionValid;     /**< whether a team ball information is valid */
  Vector2f teamPosition;        /**< ball position on the field, seen by the whole team. */

  Vector2f positionField;
  Vector2f positionRobot;
  Vector2f speedField;
  Vector2f speedRobot;
  Vector2f endPositionField;
  Vector2f endPositionRobot;

  int notSeenTime();            /**< the time of ball wasn't seen in ms(single robot) */
  float yPosWhenBallReachesOwnYAxis(); /**< y position when then ball will bast own axis */
  bool wasSeen();               /**< whether ball was seen by robot */
  bool isInOwnPenalty();        // REMAIN_TO_BE_CHECK
  bool isInOppPenalty();        // REMAIN_TO_BE_CHECK
  bool isInOwnSide();           // REMAIN_TO_BE_CHECK
  bool isInOppSide();           // REMAIN_TO_BE_CHECK
  bool isNearMiddleLine();      // REMAIN_TO_BE_CHECK

  bool isNearOppGoalLeft();     // REMAIN_TO_BE_CHECK
  bool isNearOppGoalRight();    // REMAIN_TO_BE_CHECK
  bool isNearOppGoalMiddle();   // REMAIN_TO_BE_CHECK

  bool ifStopNeed();

  void getSuitablePosition();  /**< combines ball information from both robot itself and its teammate */
  void getSuitablePosition2();  /**< combines ball information from both robot itself and its teammate */
  Vector2f getBallAngleError(Vector2f globalPosition);
};
