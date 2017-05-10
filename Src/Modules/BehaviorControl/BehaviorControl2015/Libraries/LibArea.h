/**
 * @TODO: some area information, to divide the whole soccer field into different pieces
 * @author Hanbin Wang
 * @date March 3rd
 *
 */
#pragma once

class LibArea : public LibraryBase
{
 public:
  LibArea();

  void preProcess() override;
  void postProcess() override;

  /*  @TODO This 4 vars don't work @wzf */
//  Vector2f fieldCoverageTarget;
//  Vector2f globalPatrolTarget;
//  bool fieldCoverageTargetVaild;
//  bool globalPatrolTargetValid;
  bool nearOppGoalLeft(float x, float y);
  bool nearOppGoalRight(float x, float y);
  bool nearOppGoalMiddle(float x, float y);
  
  //@LiShu 可以利用机器人到可探测边界的距离，防止机器人离开场地。
  //当机器人距离边界最大距离小于某个阈值时，可令机器人转向。
  //turnSuggest表示机器人即将出界时，应该的转向。
  enum class TurnSuggest 
  {
    NONE,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_OVER
  };

  float minDistanceToFieldBorder();
  int minDistanceToFieldBorderPoint();
  float maxDistanceToFieldBorder();
  int maxDistanceToFieldBorderPoint();
  TurnSuggest turnSuggest();

};
