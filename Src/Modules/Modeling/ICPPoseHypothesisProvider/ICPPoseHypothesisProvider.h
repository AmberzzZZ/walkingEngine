/**
 * @file ICPPoseHypothesisProvider.h
 *
 * Declaration of a module that uses recent field feature obervations
 * and combines them to an alternative pose of the robot.
 *
 * @author Zeng Zhiying
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Modeling/ICPPoseHypothesis.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/OwnSideModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"
#include "Representations/Perception/FieldPercepts/IntersectionsPercept.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/FieldFeatures/GoalFeature.h"
#include "Representations/Perception/FieldFeatures/GoalFrame.h"
#include "Representations/Perception/FieldFeatures/MidCircle.h"
#include "Representations/Perception/FieldFeatures/MidCorner.h"
#include "Representations/Perception/FieldFeatures/OuterCorner.h"
#include "Representations/Perception/FieldFeatures/PenaltyArea.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/RingBuffer.h"

#define IMPROVEMENT_THRESHOLD 20 * 20
#define MAX_ITERATIONS 12
#define ICP_LOST -1
#define LOCALISED 100 * 100

enum TargetType { POINT, VERT_LINE, HOR_LINE };

MODULE(ICPPoseHypothesisProvider,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FieldLines),
  REQUIRES(FieldLineIntersections),
  REQUIRES(PenaltyMarkPercept),
  REQUIRES(GroundContactState),
  REQUIRES(MidCircle),
  REQUIRES(MidCorner),
  REQUIRES(MotionInfo),
  REQUIRES(OuterCorner),
  REQUIRES(Odometer),
  REQUIRES(OwnSideModel),
  REQUIRES(PenaltyArea),
  USES(RobotPose),
  PROVIDES(ICPPoseHypothesis),
  DEFINES_PARAMETERS(
  {,
    (Vector2f)(Vector2f(0.02f, 0.04f)) robotRotationDeviationInStand, /**< Deviation of the rotation of the robot's torso */
	(Vector2f)(Vector2f(0.02f, 0.06f)) robotRotationDeviation,/**< Deviation of the rotation of the robot's torso */
	(float)(500.f) intersectionAssociationDistance,
	(float)(400.f) lineAssociationCorridor, /**< The corridor used for relating seen lines with field lines. */
  }),
});

/**
 * @class ICPPoseHypothesisProvider
 *
 * Computes a new robot pose
 */
class ICPPoseHypothesisProvider : public ICPPoseHypothesisProviderBase
{

public:

	/**
	* A field line
	*/
	class FieldLine
	{
		public:
		Vector2f start; /**< The starting point of the line. */
		Vector2f end; /**< The ending point of the line. */
		Vector2f dir; /**< The normalized direction of the line (from starting point). */
		float length; /**< The length of the line. */
		bool vertical; /**< Whether this is a vertical or horizontal line. */
	};

	ICPPoseHypothesisProvider();
	~ICPPoseHypothesisProvider();

	void update(ICPPoseHypothesis& icpPoseHypothesis);

    int localise(const PenaltyArea& penaltyArea, const MidCircle& midCircle, const MidCorner& midCorner, const OuterCorner& outerCorner,
    		     const FieldLineIntersections& intersectionsPercept, const FieldLines& linePercept, const PenaltyMarkPercept& penaltyMarkPercept, const FrameInfo frameInfo,
				 Pose2f robotPose, Vector3f covariance, const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix);

    void reset();

    void associateFeatures(int association, const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix);

    void preprocessObservation(const PenaltyArea& penaltyArea, const MidCircle& midCircle, const MidCorner& midCorner, const OuterCorner& outerCorner,
    		const FieldLineIntersections& intersectionsPercept, const FieldLines& linePercept, const PenaltyMarkPercept& penaltyMarkPercept, const FrameInfo frameInfo);

    void addToPointCloud(Vector2f source, Vector2f target, float dist, float weight, TargetType type);

    void solve(const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix);

    void calcMSE(void);

    void iterate(void);

    Vector2f transform(const Vector2f& point, float tx, float ty, float theta);

    Matrix2f getCovOfPointInWorld(const Vector2f& pointInWorld2, float pointZInWorld, const MotionInfo& motionInfo,
    const CameraMatrix& cameraMatrix) const;

    Matrix2f getCovOfCircle(const Vector2f& circlePos, float centerCircleRadius, const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix) const;

    bool getAssociatedIntersection(const Pose2f& robotPose, const FieldLineIntersections::Intersection& intersection, Vector2f& associatedIntersection) const;

    const FieldLine* getPointerToAssociatedLine(const Pose2f robotPose, const Vector2f& start, const Vector2f& end) const;

    float getSqrDistanceToLine(const Vector2f& base, const Vector2f& dir, float length, const Vector2f& point) const;

    bool intersectLineWithLine(const Vector2f& lineBase1, const Vector2f& lineDir1, const Vector2f& lineBase2, const Vector2f& lineDir2, Vector2f& intersection) const;

    float getSqrDistanceToLine(const Vector2f& base, const Vector2f& dir, const Vector2f& point) const;

    void getCloestCorner(const Pose2f& robotPose, const FieldLineIntersections::Intersection& intersection, Vector2f& associatedCorner) const;

private:
    int iteration;
    int minIterations;
    float mse;              //mean squared distance error
    int N;                  //number of points used on this iteration
    Pose2f robotpose;
    std::vector<PenaltyArea> penaltyAreas;
    std::vector<MidCircle> midCircles;
    std::vector<MidCorner> midCorners;
    std::vector<OuterCorner> outerCorners;
    std::vector<FieldLines::Line> lines;
    std::vector<Vector2f> penaltymarks;
    std::vector<FieldLineIntersections::Intersection> intersections;
    std::vector<Vector2f> sourcePoints;
    std::vector<Vector2f> targetPoints;
    std::vector<float> distances;
    std::vector<float> weights;
    std::vector<TargetType> targetType;
    std::vector<FieldLine> verticalFieldLines;   /**< Relevant field lines  */
    std::vector<FieldLine> horizontalFieldLines; /**< Relevant field lines  */
    std::vector< Vector2f > xIntersections;
    std::vector< Vector2f > lIntersections;
    std::vector< Vector2f > tIntersections;
    bool x_known;
    bool y_known;
    bool z_known;
    bool nonLineFeature;
    Vector3f cov;
};


