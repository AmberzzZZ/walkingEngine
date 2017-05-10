/**
 * @file LibCommon.h
 */

class LibCommon : public LibraryBase
{
 public:
  /** Constructor for initializing all members*/
  LibCommon(); 

  void preProcess() override;
  void postProcess() override;

  bool between(float value, float min, float max);

  float fromDegrees(float degrees);

  float fromRad(float rad);

  float angleToPose(Pose2f);

  float radToPose(Pose2f);

  float getDistanceSqr(const Vector2f&, const Vector2f&);

  void stepClipped(float& forward, float& left, Angle& turn);

  int gameState = 0;
  int lastGameState = 0;
  unsigned int kickOffTime = 0;
  bool isKickOff = false;

  bool isKickOffLimit = false;

  SupporterBehavior::KickOffDirection kickOffDirection;

};
