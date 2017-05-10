/**
 * @file LibGoals.h
 * @author Hanbin Wang
 * @date Feb 27, 2014
 */

class LibGoals : public LibraryBase
{
 public:
  /** Constructor for initializing all members*/
  LibGoals();

  void preProcess() override;
  void postProcess() override;

  //@Usage : goal.angleToGoal
  float angleToGoal;
  float freeLeftEndX;
  float freeLeftEndY;
  float freeRightEndX;
  float freeRightEndY;
  bool hasFreePart;
  bool goalWasSeenFlag;  //@han:used in Penalty state

  float timeSinceOnePostWasSeen();  //time since one goal post was last seen in "ms"
  float timeSinceTwoPostsWasSeen();  //time since two goal posts was last seen in "ms"
  float centerAngleOfOppFreePart();  //angle to the center of the largest free part of the opponent goal
  float angleToShoot();

  //Vector2<double> leftOppGoalPost;
  //Vector2<double> rightOppGoalPost;
};
