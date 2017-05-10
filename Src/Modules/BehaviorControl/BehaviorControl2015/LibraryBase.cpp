/**
 * The file implements the base class for all behavior libraries.
 * If a library is used by another library, it must be added here.
 * @author Thomas Röfer
 */

#include "Libraries.h"

namespace Behavior2015
{
  LibraryBase::LibraryBase ():BehaviorBase (*Libraries::theInstance),
    common (Libraries::theInstance->common),
    area (Libraries::theInstance->area),
    ball (Libraries::theInstance->ball),
    behavior (Libraries::theInstance->behavior),
    goals (Libraries::theInstance->goals),
    robot (Libraries::theInstance->robot),
    player (Libraries::theInstance->player),
    keeper (Libraries::theInstance->keeper),
    defender (Libraries::theInstance->defender),
    stabber (Libraries::theInstance->stabber),
    obstacle (Libraries::theInstance->obstacle),
    teammate (Libraries::theInstance->teammate),
    supporter(Libraries::theInstance->supporter),
    walk(Libraries::theInstance->walk)
  {
    Libraries::theInstance->libraries.push_back (this);
  }
}
