
/**
 * @file PathFinder.h
 * @author Katharina Gillmann
 * @modified by hjq
 * @date 2016.03.06
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/TeamPlayersModel.h"
//#include "Representations/Infrastructure/GameInfo.h"


class PathFinder
{
public:

  /*
  * @class Node
  * a node of a path
  */
    class Node
    {
    public:
        Vector2f position; 						/**position of the current node*/
        int indexPreviousNode; 					/** index of the previous node in the path*/
        Node() : indexPreviousNode(-1) {} 	 	/**Constructor*/
        Node(const Vector2f& position) : position(position), indexPreviousNode(-1) {}
    };

    /**
     * class Path
     * path include the position of every point and the length of path
     */
//    class Path
//    {
//    public:
//    	std::vector<Vector2f> path;
//    	float pathLength;
//    	Path() : pathLength(0.0f){}
//    };

    /**
     * @function
     * Constructor of PathFinder
     */
    PathFinder(const FieldDimensions& fieldDimensions,const RobotInfo& robotInfo,const TeamPlayersModel& teamPlayersModel) :
    theFieldDimensions(fieldDimensions),
    theRobotInfo(robotInfo),
	theTeamPlayersModel(teamPlayersModel),
    countNoPathFound(0)
    {}

    std::vector<Vector2f> path; 		/** current complete path*/
//    std::vector<Vector2f> betterPath;
    std::vector<Node> tree;				/** random search tree*/
    float pathLength = 0; 					/** length of the path*/

    const FieldDimensions& theFieldDimensions;
    const RobotInfo& theRobotInfo;
    const TeamPlayersModel& theTeamPlayersModel;

    /**
     * @function findPath
     * Find path to avoid obstacles
     */
    void findPath(const Vector2f& startOfPath, const Vector2f& endOfPath);
    /**
       * function findBetterPath
       * find better after several times
       */
//    void findBetterPath(const Vector2f& startOfPath, const Vector2f& endOfPath);
    /**
     * @function loadParameters
     * Load the parameters from pathFinder.cfg
     */
    void loadParameters();

    /**
    * @class Parameters
    * The parameters of the module
    */
    STREAMABLE(Parameters,
    {,
      (float)(200.f) stepSize,                  /**< The length of the new added edge */
      (float)(400.f) distancePathCombination,   /**<max distance between two paths for combining them > */
      (float)(400.f) distanceToObstacle,        /**<needed distance between robot and obstacle> */
      (float)(200.f) distanceCloseToObstacle,   /**<close distance of positions edge and obstacle> */
      (float)(0.3f) targetProbability,         	/**<probability for using target as random position> */
      (float)(0.7f) wayPointsProbability,      	/**<probability for using old wayPoints as random position> */
      (int)(500) countOfCycles,               	/**<count of cycles for creating one way> */
      (float)(1.5f) searchSpaceFactor,         	/**<factor for the search area in which random positions can be found>*/
      (int)(200) counterUseOldPath,           	/**<counter until old path is used if no new one was found>*/
      (bool)(false) enterPenaltyAreaInReady,    /**<flag the enables/disables the penalty area obstacle>*/
	  (float)(0.8)gravitationCoefficient,		/**Gravitational coefficient towards the target position*/
    });

  Parameters parameters; 						/**< The parameters of this PathFinder */

private:
  int countNoPathFound=0;
  std::vector<Node> completePath; 		/**the complete path found in the last run*/
  bool wayPointFlag = false;
  /**
   * @function createRandomPosition
   * Create random position for the tree
   */
  void createRandomPosition(Vector2f& randomPosition, const Vector2f& targetPosition);

  /**
   * @function cheakForObstacleNeatsPosition
   * Check whether there are some obstacles near the position
   */
  bool checkForObstaclesNearPosition(const std::vector<Node>& allObstacles, const Vector2f& position, Vector2f& positionOfObstacle,float& smallestDistance);

  /**
   * @function calculateNearestNode
   * Calculation the nearest node in the tree to the random position
   */
  void calculateNearestNode(int& indexNearestNode, const Vector2f& randomPosition);

  /**
   * @function checkForCollision
   * Check whether collision between the random position and the nearest node
   */
  bool checkForCollision(const std::vector<Node>& allObstacles, const Vector2f& nearestNode, const Vector2f& randomPosition,bool usePosition, float& distanceToObstacle, Vector2f& positionObstacle);

  /**
   * @function checkForFoundPath
   * Check whether the path is found
   */
  void checkForFoundPath(const Vector2f targetPosition,bool& foundPath);

  /**
   * @function createNewNode
   * Create new node
   */
  void createNewNode(const Vector2f randomPosition, const int indexNearestNode, const Vector2f targetPosition,
			const float GravCoefficient);

  /**
   * @function createCompletePath
   * Create complete path in the tree
   */
  void createCompletePath(const Vector2f& targetPosition);
  /**
   * @function add AvoidingPoints
   * Add avoiding points near the start or the end point
   */
  void addAvoidingPoints(const std::vector<Node>& allObstacles, std::vector<Node>& currentUsedTree, const Vector2f& position, const Vector2f& positionOfObstacle,
		  bool nearestObstacle, const Node& target, const bool startIsUsed);

  /**
   * function isCollisionDirect
   * to detect whether there are no obstacles between start and end position
   */
  bool isCollisionDirect(const std::vector<Node>& allObstacles,const Vector2f startOfPath, const Vector2f endOfPath,float& distanceToObstacle, Vector2f& positionObstacle);
  /**
   *  @function pathSmooth
   *  smooth the path, delete the invalid node in path;
   */
  void pathSmooth(const std::vector<Node>& allObstacles);

};
