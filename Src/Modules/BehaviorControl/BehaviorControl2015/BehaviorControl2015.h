/**
 * @file BehaviorControl2015.h
 * Declaration of the base class of the C-based state machine behavior control module.
 * @author Thomas Röfer
 * @author Tim Laue
 */
 
#pragma once

#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/RoleBehavior.h"
#include "Representations/BehaviorControl/SPLStandardBehaviorStatus.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/PenaltyShootParameter.h"	//@tjark_easy
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Communication/TeammateData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeammateReliability.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/MotionControl/ArmKeyFrameEngineOutput.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/FieldPercepts/GoalPercept2017.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/PlayersPercepts/PlayersPercept.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootContactModel.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Random.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Module/Module.h"
#include "Tools/Team.h" //@DXG
#include "Tools/PathFinder.h"
//#include "Representations/Perception/TJArkVision.h"
#include "Representations/Modeling/LocalizationTeamBall.h"
#include <algorithm>
#include <limits>
#include <sstream>
#include <iostream>
#include "Platform/SystemCall.h"

MODULE(BehaviorControl2015,
	{,
	REQUIRES(ArmContactModel),         REQUIRES(ArmKeyFrameEngineOutput),
	REQUIRES(BallModel),               REQUIRES(CameraCalibration),
	REQUIRES(CameraInfo),              REQUIRES(CameraMatrix),
	REQUIRES(DamageConfigurationBody), REQUIRES(FallDownState),
    REQUIRES(ImageCoordinateSystem),
	REQUIRES(FieldBoundary),           REQUIRES(FieldDimensions),
	REQUIRES(FootContactModel),        REQUIRES(FrameInfo),
	REQUIRES(GameInfo),                REQUIRES(GoalPercept2017),
  REQUIRES(GroundContactState),      REQUIRES(HeadJointRequest),
  REQUIRES(HeadLimits),              REQUIRES(Image),
//  REQUIRES(TJArkVision),
  REQUIRES(JointAngles),             REQUIRES(JointRequest),
  REQUIRES(KeyStates),               REQUIRES(KickEngineOutput),
  REQUIRES(LinesPercept),             REQUIRES(MotionInfo),
  REQUIRES(MotionSelection),         REQUIRES(ArmMotionSelection),
  REQUIRES(ObstacleModel),           REQUIRES(Odometer),
   REQUIRES(PlayersPercept),
  REQUIRES(OpponentTeamInfo),        REQUIRES(OwnTeamInfo),
  REQUIRES(PenaltyShootParameter),	 REQUIRES(RawGameInfo),
	REQUIRES(RobotDimensions),	       REQUIRES(RobotInfo),
  REQUIRES(RobotModel),              REQUIRES(RobotPose),
  REQUIRES(Role),                    REQUIRES(SideConfidence),
  REQUIRES(TeamBallModel),           REQUIRES(TeammateData),
  REQUIRES(TeammateReliability),     REQUIRES(TeamPlayersModel),
  REQUIRES(TorsoMatrix),             REQUIRES(WalkingEngineOutput),
	REQUIRES(Whistle),	               REQUIRES(ActivationGraph),
  REQUIRES(LocalizationTeamBall),     REQUIRES(InertialData),
	PROVIDES(ActivationGraph),         PROVIDES(ArmMotionRequest),
	PROVIDES(BehaviorLEDRequest),      PROVIDES(BehaviorMotionRequest),
	PROVIDES(BehaviorStatus),          PROVIDES(HeadMotionRequest),
	PROVIDES(SPLStandardBehaviorStatus),
  PROVIDES(RoleBehavior),
	});

namespace Behavior2015
{
  /**
   * Container for references to representations modified by the behavior.
   */
  class BehaviorData
  {
  public:
    ActivationGraph & theActivationGraph;
    BehaviorLEDRequest & theBehaviorLEDRequest;
    BehaviorStatus & theBehaviorStatus;
    HeadMotionRequest & theHeadMotionRequest;
    ArmMotionRequest & theArmMotionRequest;
    BehaviorMotionRequest & theMotionRequest;
    SPLStandardBehaviorStatus & theSPLStandardBehaviorStatus;
    RoleBehavior & theRoleBehavior;

    BehaviorData (ActivationGraph & theActivationGraph,
		  BehaviorLEDRequest & theBehaviorLEDRequest,
		  BehaviorStatus & theBehaviorStatus,
		  HeadMotionRequest & theHeadMotionRequest,
		  ArmMotionRequest & theArmMotionRequest,
		  BehaviorMotionRequest & theMotionRequest,
      SPLStandardBehaviorStatus & theSPLStandardBehaviorStatus,
      RoleBehavior & theRoleBehavior):theActivationGraph (theActivationGraph),
      theBehaviorLEDRequest (theBehaviorLEDRequest),
      theBehaviorStatus (theBehaviorStatus),
      theHeadMotionRequest (theHeadMotionRequest),
      theArmMotionRequest (theArmMotionRequest),
      theMotionRequest (theMotionRequest), theSPLStandardBehaviorStatus (theSPLStandardBehaviorStatus),
      theRoleBehavior (theRoleBehavior)
    {
    }
  };

  /**
   * Common base class for behavior options and libraries.
   */
  class BehaviorBase:public BehaviorControl2015Base, public BehaviorData
  {
  private:
    void update (ActivationGraph &)
    {
    }
    void update (BehaviorLEDRequest &)
    {
    }
    void update (BehaviorMotionRequest &)
    {
    }
    void update (BehaviorStatus &)
    {
    }
    void update (HeadMotionRequest &)
    {
    }
    void update (ArmMotionRequest &)
    {
    }
    void update (SPLStandardBehaviorStatus &)
    {
    }
    void update(RoleBehavior &)
    {
    }

  public:
    using BehaviorData::theActivationGraph; /**< Use the non-const version. */

    /**
     * Constructor.
     * Note that the constructor uses the default constructor of the base class.
     * @param base The behavior base class some attributes are copied from.
     * @param behaviorData The data modified by the behavior.
     */
    BehaviorBase (const BehaviorControl2015Base & base, BehaviorData & behaviorData):BehaviorData (behaviorData)
    {
    }

    /**
     * Copy constructor.
     * Note that the constructor uses the default constructor of the base class, because
     * the attributes of the base class should not be copied.
     * @param other The object that is copied.
     */
    BehaviorBase (const BehaviorBase & other):BehaviorData (other)
    {
    }
  };
}
