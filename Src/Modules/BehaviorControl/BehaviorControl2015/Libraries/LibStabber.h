#pragma once

class LibStabber : public LibraryBase
{
 public:
  /** Constructor for initializing all members*/
  LibStabber();

  void preProcess() override;
  void postProcess() override;

  Pose2f StabberPosition();  //relative
  Pose2f StabberPositionWhenBallNearOurGoal();
  Pose2f StabberPositionWhenBallNearOppoGoal();
  Pose2f StabberPositionWhenBallFarFromGoal();

  BallModel getBallPosition();

  bool isTooCloseToStriker();
  
  bool isPointOccupied(Vector2f point,std::vector<Vector2f> obstacles);  //whether point is occupied / 2017.2.27
  
  
};
