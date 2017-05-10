/**
 * The file declares a class that instantiates all libraries.
 * @author Thomas Röfer
 */

#pragma once

#include "LibraryBase.h"

namespace Behavior2015
{
#include "Libraries/LibCommon.h"
#include "Libraries/LibArea.h"
#include "Libraries/LibAlignFuzzy.h"
#include "Libraries/LibBall.h"
#include "Libraries/LibBehavior.h"
#include "Libraries/LibGoals.h"
#include "Libraries/LibRobot.h"
#include "Libraries/LibPlayer.h"
#include "Libraries/LibKeeper.h"
#include "Libraries/LibDefender.h"
#include "Libraries/LibStabber.h"
#include "Libraries/LibObstacle.h"
#include "Libraries/LibTeammate.h"
#include "Libraries/LibSupporter.h"
#include "Libraries/LibWalk.h"
#include "Libraries/LibWalkToTarget_SiQiang.h"
#include "Libraries/LibStriker.h"

  class Libraries:public BehaviorBase
  {
  private:
    static thread_local Libraries *theInstance;
    std::vector < LibraryBase * >libraries;
					 /**< All the member libraries of this class. */

  public:
    LibCommon common;
				   /**< Contains miscellaneous helper methods */
    LibArea area;
    LibBall ball;
    LibBehavior behavior;
    LibGoals goals;
    LibRobot robot;
    LibPlayer player;
    LibKeeper keeper;
    LibDefender defender;
    LibStabber stabber;
    LibObstacle obstacle;
    LibTeammate teammate;
    LibSupporter supporter;
    LibWalk walk;
    LibAlignFuzzy alignFuzzy;
    LibWalkToTarget walkToTarget;
    LibStriker striker;

    Libraries (const BehaviorControl2015Base & base, BehaviorData & behaviorData);

    /**
     * Assignment operator, because the standard operator is not accepted by the compiler.
     * @param other The instance that is cloned.
     */
    void operator= (const Libraries & other);

    /** Calls the preProcess() method of each member library */
    void preProcessLibraries ();

    /** Calls the postProcess() method of each member library */
    void postProcessLibraries ();

    friend class LibraryBase;
  };
}
