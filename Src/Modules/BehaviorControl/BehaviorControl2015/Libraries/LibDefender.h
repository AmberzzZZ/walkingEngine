class LibDefender : public LibraryBase
{
 public:
  LibDefender();

  void preProcess() override;

  void postProcess() override;

  float yOnGoal;
  
  Vector2f ballPositionField;
  Vector2f ballPositionRobot;
  Vector2f ballSpeedField;
  Vector2f ballSpeedRobot;
  Vector2f ballEndPositionField;
  Vector2f ballEndPositionRobot;
  float yIfBallPassOwnAxis;
  bool useTeamBall;
  bool lastIsLeft = false;
  
  Geometry::Line defenderLine = Geometry::Line(Vector2f(-3200, 0),
                                               Vector2f(0, 1));
  Vector2f defenderPositionWhenKeeperPenalized();  //站中间
  Vector2f defenderPositionWhenKeeperNotPenalized();  //站左边
  Vector2f supporterPositionWhenKeeperNotPenalized();  //站右边
  Vector2f defenderPositionCoorperateWithKeeperAndSupporter(bool left);
  float yIfBallCanPassGroundLine();  //球朝自家球门运动时，估测球运动到自家球门时的y值
  bool ifStopBallNeeded();  //球的运动轨迹会进自家球门时为true
  bool ifStopBallNeeded2();  //球的运动轨迹会进自家球门时为true
  bool toStopBall();  //ifStopBallNeeded()为true，且球离自己的距离足够近时为true
  bool toStopBall2();  //ifStopBallNeeded()为true，且球离自己的距离足够近时为true
  Pose2f defenderDefencePosition();  //相对位置
  Pose2f supporterDefencePosition();  //相对位置
  bool ifSpreadNeeded();  //toStopBall()为ture，且球会从自己身旁经过时为true
  bool ifSpreadNeeded2();  //toStopBall()为ture，且球会从自己身旁经过时为true
  bool robotAndBallCloseEnough();  //球离自己足够近时为true
  bool robotAndBallCloseEnough2();  //球离自己足够近时为true
  Vector2f globalSpeed(Vector2f relativeSpeed);
  
  Pose2f defenderDefencePosition2();
  /**
   * @function get ball position on field using both own observation and team info
   * @return ball poition on field
   */
  Vector2f getBallPositionField();
  
  /**
   * @function get ball position to robot using both own observation and team info
   * @return ball poition to robot
   */
  Vector2f getBallPositionRobot();

  /**
   * @function get ball speed on field
   * @return ball speed on field
   */
  Vector2f getBallSpeedField();
  
  /**
   * @function get ball speed to robot
   * @return ball speed to robot
   */
  Vector2f getBallSpeedRobot();
  
  /**
   * @function get ball end point on field
   * @return ball end point on field
   */
  Vector2f getBallEndPointField();
  
  /**
   * @function get ball end point to robot
   * @return ball end point to robot
   */
   Vector2f getBallEndPointRobot();
  /**
   * @function get ball position when it comes owm axis
   * @return point if it will pass own axis
   * */
   float getYIfBallPassOwnAxis();

};
