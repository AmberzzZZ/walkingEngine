/************************************************
 * @file   LibKeeper.h
 * @author XiangWei Wang
 * @Modified	Li Shu @ Sep.12
 * @date   2014 25th Mar
 * @usage  KeeperParameter
 ************************************************/

#pragma once

class LibKeeper : public LibraryBase
{
 public:
  LibKeeper();

  void preProcess() override;

  void postProcess() override;
  bool initial_flag = true;  //@wangxw @2014 03 13
  float positionOfKeeper;
  std::vector<Vector2f> ballPath;
  float yPosWhenBallReachesOwnYAxis();
  float yPosWhenBallReachesOwnYAxis2();
  float toStandInTheWay();
  float toStandInTheWayGlobal();
  float yBallPassRobot;
  float yOnGoal;
  
  Vector2f ballPositionField;
  Vector2f ballPositionRobot;
  Vector2f ballSpeedField;
  Vector2f ballSpeedRobot;
  Vector2f ballEndPositionField;
  Vector2f ballEndPositionRobot;
  bool useTeamBall;
  //Vector2<> globalBallSpeed;
  /*********************************************************
   *
   * @fun   calculate the desired transition in y direction on global
   *        assume the robot need not move in x direction
   * @return the value said above

   * ******************************************************/
  float getPositionOfKeeper();
  float getPositionOfKeeper2();
  /*********************************************************
   *
   * @fun   calculate the desired transition in y direction on global
   *
   * @para xRobot the desired transition in x direction on gloabal
   * @return the value said above

   * ******************************************************/

  float getPositionOfKeeper(float xRobot);
  float getPositionOfKeeper2(float xRobot);
  /*********************************************************
   *
   * @fun   calculate the desired Position of keeper relative
   *        it have taken getPositionOfKeeperOnArc() in account
   *
   * @para xRobot the desired transition in x direction on gloabal
   * @para flag if flag is true, xRobot is set by ourself; if flag is false, robot need not to
   *       move in x direction
   * @return the desired pose of the robot

   * ******************************************************/
  Pose2f getPositionOfKeeper(float xRobot, bool flag);  
  /*********************************************************
   *
   * @fun   calculate the desired Position of keeper relative
   *        it have taken getPositionOfKeeperOnArc() in account
   *
   * @para xRobot the desired transition in x direction on gloabal
   * @para flag if flag is true, xRobot is set by ourself; if flag is false, robot need not to
   *       move in x direction
   * @return the desired pose of the robot

   * ******************************************************/
  Pose2f getPositionOfKeeper2(float xRobot, bool flag);
  /*********************************************************
   *
   * @fun   calculate the threshold above which the robot should move
   *
   * @para yRobot the abs value of the robot transition in y direction on global
   * @return the threshold

   * ******************************************************/
  float threshold(float yRobot);	//0~0.2
  /*********************************************************
   *
   * @fun   calculate the desired positon that the keeper should go on the arc
   *
   * @return the value said above

   * ******************************************************/
  Pose2f getPositionOfKeeperOnArc();
  Pose2f getPositionOfKeeperOnArc2();
  /*********************************************************
   *
   * @fun   to check if the keeper should kick the ball faraway
   * @return if ture, kick the ball faraway

   * ******************************************************/
  bool keeperKickBallFarAway();
  /*********************************************************
   *
   * @fun   calculate the position where the keeper should stand for defence
   * @return the position where the keeper should stand relative
   * it is the final position tack everything into account

   * ******************************************************/
  Pose2f keeperDefencePosition(); 
 /*********************************************************
   *
   * @fun   calculate the position where the keeper should stand for defence
   * @return the position where the keeper should stand relative
   * it is the final position tack everything into account

   * ******************************************************/
  Pose2f keeperDefencePosition2();
  /*********************************************************
   *
   * @fun   calculate the y value of the ball when it pass the ground line on global
   *        it is used to judge if the ball will come to our goal if we do not take steps
   * @return the value said above

   * ******************************************************/
  float yIfBallCanPassGroundLine();
  /********************************************************
   * @fun use the value calculated by the functiion yIfBallCanPassGroundLine to decide whether to
   *      save the ball
   * @return if true,saved;
   * ******************************************************/
  bool ifSaveNeeded();
  bool ifSaveNeeded2();

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
  
  bool ballDangerous();
  bool ballDangerous2();

  bool ifDiveNeeded();	//>300
  bool ifDiveNeeded2();	//>300

  //bool diveAble();

  bool turnAround();	//球场边界离自己太近

  bool turnAroundOver();	//球场边界离自己太远
};
