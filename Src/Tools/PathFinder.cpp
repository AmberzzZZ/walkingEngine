/**
 * @file Tools/PathFinder.cpp
 * Implementation of a class that provides methods for planing a path.
 * @author Katharina Gillmann
 * @modified by hjq
 * @date 2016.03.06
 */

#include "Tools/PathFinder.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Random.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/InStreams.h"
#include <cstdlib>
#include <cfloat>
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

/*import the parameters data*/
void PathFinder::loadParameters()
{
	InMapFile stream("pathFinder.cfg");
	if(stream.exists())
	{
		stream>>parameters;
	}
}

void PathFinder::findPath(const Vector2f& startOfPath,const Vector2f& endOfPath)
{

	bool foundPath = false;
	wayPointFlag = false;
	tree.clear();
	path.clear();
	pathLength = 0.0f;

	Node startPosition(startOfPath);
	Node endPosition(endOfPath);
	vector<Node> allObstacles; // puts all obstacles into one vector
	tree.push_back(startPosition);
	return;

	for(Obstacle obstacle:theTeamPlayersModel.obstacles)
	{
		allObstacles.push_back(Node(obstacle.center));
	}


//	for(vector<Node>::const_iterator i = allObstacles.begin(), end = allObstacles.end(); i!=end; ++i)
//	{
//		cout<<i->position.x()<<","<<i->position.y()<<endl;
//	}
//	cout<<endl;

	float distanceToObstacle;
	Vector2f positionObstacle;
	bool collision = isCollisionDirect(allObstacles,startOfPath,endOfPath,distanceToObstacle,positionObstacle);


	if (!collision)
	{
		foundPath = true;
//		tree.push_back(endPosition);
	}


	int countOfCycles = 0;
	while(!foundPath && countOfCycles <= parameters.countOfCycles)
	{
		Vector2f randomPosition;
		createRandomPosition(randomPosition, endOfPath);
		int IndexNearestNode = 0;
		float distanceToObstacle;
		Vector2f positionObstacle;
		calculateNearestNode(IndexNearestNode, randomPosition); // calculates the nearest node of the tree towards the random position
		bool collides;
		collides = checkForCollision(allObstacles,tree[IndexNearestNode].position, randomPosition, false, distanceToObstacle, positionObstacle); // checks if the new node collision with an obstacle
		if (!collides)
		{
			createNewNode(randomPosition, IndexNearestNode,endOfPath,parameters.gravitationCoefficient); // creates the new node
			checkForFoundPath(endOfPath, foundPath); // checks if a complete path was found
		}
		countOfCycles++;
	}

	if (foundPath)
	{
		countNoPathFound = 0;
		createCompletePath(endOfPath);
	}
	//use old path if no new path was found
	else if (countNoPathFound < parameters.counterUseOldPath && completePath.size() != 0)
	{

		countNoPathFound++;
		vector<PathFinder::Node> temp = completePath;
		int index = -1;
		float currentDistance = FLT_MAX;
		int size = completePath.size() <= 11 ? completePath.size()-2 : 10;
		for (int i = 0; i <= size; i++)
		{
			Geometry::Line line((completePath.begin() + i)->position, (completePath.begin() + i + 1)->position - (completePath.begin() + i)->position);
			Vector2f rotatedDirection = line.direction.rotateLeft();

			Vector2f positionRelToBase = startOfPath - line.base;
			//relevant means the startOfPath in the left plane divided by the rotatedDirection;
			bool firstNodeRelevant = (rotatedDirection.x() * positionRelToBase.y() - positionRelToBase.x() * rotatedDirection.y()) >=0;
			float intersect = Geometry::getDistanceToEdge(line,startOfPath);
			if (intersect < currentDistance)
			{
				if (firstNodeRelevant)
				{
					index = i;
				}
				else
				{
					index = i+1;
				}
				currentDistance = intersect;
			}
		}

		completePath.clear();
		completePath.push_back(startOfPath);
		for (unsigned int i = index; i < temp.size(); i++)
		{
			completePath.push_back(temp[i]);
		}
	}
	else // no path is available
	{

		completePath.clear();
		completePath.push_back(startOfPath);
		completePath.push_back(endOfPath);
	}


	for (unsigned int i = 0; i < completePath.size(); i++)
	{
		path.push_back(completePath[i].position);
		if(i != 0)
		{
			pathLength += (completePath[i].position - completePath[i - 1].position).norm();
		}
	}

	pathSmooth(allObstacles);

	for (unsigned int i = 0; i < path.size(); i++)
	{
		if(i != 0)
		{
			pathLength += (path[i] - path[i - 1]).norm();
		}
	}


}

//void PathFinder::findBetterPath(const Vector2f& startOfPath, const Vector2f& endOfPath)
//{
//	vector<Path> severalPath;
//	Path temp;
//	float currentLength = 0.0f;
//	for (int i = 0; i<3; i++)
//	{
//		temp.path.clear();
//		findPath(startOfPath,endOfPath);
//		for(Vector2f tempPosition : path)
//		{
//			temp.path.push_back(tempPosition) ;
//		}
//		temp.pathLength = pathLength;
//		severalPath.push_back(temp);
//	}
//	currentLength = severalPath.begin()->pathLength;
//
//	for (Path tempPath : severalPath)
//	{
//		if(tempPath.pathLength <= currentLength)
//		{
//			currentLength = tempPath.pathLength;
//			for(Vector2f tempPosition : tempPath.path)
//			{
//				betterPath.push_back(tempPosition) ;
//			}
//			pathLength = currentLength;
//		}
//	}
//}

bool PathFinder::checkForCollision(const vector<Node>& allObstacles, const Vector2f& nearestNode, const Vector2f& randomPosition,
		bool usePosition, float& distanceToObstacle, Vector2f& positionObstacle)
{
	bool collides = false;

	float currentDistance = parameters.distanceToObstacle;
	Vector2f newPosition;
	if((nearestNode - randomPosition).norm() < parameters.stepSize || usePosition)
	{
		newPosition = randomPosition;
//		cout<<"usePosition is true!"<<endl;
	}
	else
	{
		Vector2f tempPosition = randomPosition - nearestNode;
		newPosition = nearestNode + tempPosition.normalize(parameters.stepSize);
	}

	//if new node is outside the field
	if(std::abs(newPosition.x()) > (theFieldDimensions.xPosOpponentGroundline + (theFieldDimensions.xPosOpponentFieldBorder - theFieldDimensions.xPosOpponentGroundline)/2)||
			std::abs(newPosition.y()) > (theFieldDimensions.yPosLeftSideline + (theFieldDimensions.yPosLeftFieldBorder - theFieldDimensions.yPosLeftSideline)/2))
	{
		collides = true;
		return collides;
	}

	//path should be around the own penalty area for avoiding illegal defenders(only for field players)
	if (newPosition.x() < theFieldDimensions.xPosOwnPenaltyArea + 200 && std::abs(newPosition.y()) < theFieldDimensions.yPosLeftPenaltyArea + 200 &&
			theRobotInfo.number != 1)
	{
		collides = true;
		return collides;
	}


	//calculate the line between the nearestNode and the newPosition
	Geometry::Line segmentOfPath(nearestNode, newPosition - nearestNode);

	Vector2f directionNearestToNew = newPosition - nearestNode;
//	Vector2f directionNewToNearest = nearestNode - newPosition;
	for (std ::vector<Node>::const_iterator i = allObstacles.begin(), end = allObstacles.end(); i != end; ++i)
	{
		Vector2f obstacleRelToNearest = i->position - nearestNode;
//		Vector2f obstacleRelToNew = i->position - newPosition;
		bool obstaclesRelevant = (directionNearestToNew.transpose() * obstacleRelToNearest > 0); //&& (directionNewToNearest.transpose() * obstacleRelToNew > 0);
		float obstacleDistance = Geometry::getDistanceToEdge(segmentOfPath, i->position);
		if (obstacleDistance < currentDistance && obstaclesRelevant)
		{
			currentDistance = obstacleDistance;
			positionObstacle = i->position;
			collides = true;
		}
	}
	distanceToObstacle = currentDistance;
	return collides;
}

bool PathFinder::isCollisionDirect(const vector<Node>& allObstacles,const Vector2f startOfPath, const Vector2f endOfPath,float& distanceToObstacle, Vector2f& positionObstacle)
{
	bool collides = false;
	float currentDistance = parameters.distanceToObstacle;
	if(std::abs(endOfPath.x()) > (theFieldDimensions.xPosOpponentGroundline + (theFieldDimensions.xPosOpponentFieldBorder - theFieldDimensions.xPosOpponentGroundline)/2)||
			std::abs(endOfPath.y()) > (theFieldDimensions.yPosLeftSideline + (theFieldDimensions.yPosLeftFieldBorder - theFieldDimensions.yPosLeftSideline)/2))
	{
		collides = true;
		return collides;
	}

	//path should be around the own penalty area for avoiding illegal defenders(only for field players)
	if (endOfPath.x() < theFieldDimensions.xPosOwnPenaltyArea + 200 && std::abs(endOfPath.y()) < theFieldDimensions.yPosLeftPenaltyArea + 200 &&
			theRobotInfo.number != 1)
	{
		collides = true;
		return collides;
	}
	Geometry::Line segmentOfPath(startOfPath, endOfPath - startOfPath);
	Vector2f directionBaseToTarget = endOfPath - startOfPath;
	Vector2f directionTargetToBase = startOfPath - endOfPath;

	for (std ::vector<Node>::const_iterator i = allObstacles.begin(), end = allObstacles.end(); i != end; ++i)
	{
		Vector2f obstacleRelToBase = i->position - startOfPath;
		Vector2f obstacleRelToTarget = i->position - endOfPath;

		bool obstaclesRelevant = (directionBaseToTarget.transpose() * obstacleRelToBase > 0) && (directionTargetToBase.transpose() * obstacleRelToTarget > 0);
		float obstacleDistance = Geometry::getDistanceToEdge(segmentOfPath, i->position);
		if (obstacleDistance < currentDistance && obstaclesRelevant)
		{
			currentDistance = obstacleDistance;
			positionObstacle = i->position;
			collides = true;
		}
	}
	distanceToObstacle = currentDistance;
	return collides;
}


void PathFinder::createRandomPosition(Vector2f& randomPosition, const Vector2f& targetPosition)
{
	float probabilityTarget = parameters.targetProbability;
	float probalilityWayPoint = parameters.wayPointsProbability;
	float rand = Random::bernoulli();
	if (rand <= probabilityTarget)
	{
		randomPosition = targetPosition;
	}
	else if (rand <= probalilityWayPoint && completePath.size() !=0 )
	{
		int rand =random((short)completePath.size());
		randomPosition = (completePath.begin()+rand)->position;
		wayPointFlag = true;
	}
	else
	{
		randomPosition.x() = Random::bernoulli() * ((theFieldDimensions.xPosOpponentFieldBorder * parameters.searchSpaceFactor)
				- (theFieldDimensions.xPosOwnFieldBorder * parameters.searchSpaceFactor))
				+ (theFieldDimensions.xPosOwnFieldBorder * parameters.searchSpaceFactor);
		randomPosition.y() = Random::bernoulli() * ((theFieldDimensions.yPosLeftFieldBorder * parameters.searchSpaceFactor)
				- (theFieldDimensions.yPosRightFieldBorder * parameters.searchSpaceFactor))
				+ (theFieldDimensions.yPosRightFieldBorder * parameters.searchSpaceFactor);
	}
}

void PathFinder::calculateNearestNode(int& indexNearestNode, const Vector2f& randomPosition)
{
	float distance = FLT_MAX;
	for (unsigned int i = 0; i < tree.size(); i++)
	{
		float distanceBetweenPositions = (randomPosition - tree[i].position).norm();
		if(distanceBetweenPositions < distance)
		{
			indexNearestNode = i;
			distance = distanceBetweenPositions;
		}
	}
}

void PathFinder::createNewNode(const Vector2f randomPosition, const int indexNearestNode, const Vector2f targetPosition,
		const float GravCoefficient)
{
	Vector2f positionNewNode;
	Vector2f randomFun;
	Vector2f targetFun;
//	bool useTargetFun = wayPointFlag? false:true;

	if ((tree[indexNearestNode].position - randomPosition).norm() < parameters.stepSize)
	{
		randomFun = randomPosition - tree[indexNearestNode].position;
	}
	else
	{
		Vector2f tempRandomNode = randomPosition - tree[indexNearestNode].position;
		randomFun = tempRandomNode.normalize(parameters.stepSize);
	}
	Vector2f tempTargetNode = targetPosition - tree[indexNearestNode].position;
	targetFun = tempTargetNode.normalize(parameters.stepSize*GravCoefficient);

	if(wayPointFlag)
		positionNewNode = tree[indexNearestNode].position+randomFun;
	else
		positionNewNode = tree[indexNearestNode].position+randomFun+targetFun;


	Node newNode(positionNewNode);
	newNode.indexPreviousNode = indexNearestNode;
	tree.push_back(newNode);
}

void PathFinder::checkForFoundPath(const Vector2f targetPosition,bool& foundPath)
{

	float stepSizeSqr = sqr(parameters.stepSize);
	float distance = (tree.back().position - targetPosition).squaredNorm();
	if (distance < stepSizeSqr)
	{
		foundPath = true;
//		Node backNode;
//		backNode.indexPreviousNode = tree.size()-1;
//		backNode.position = targetPosition;
//		tree.push_back(backNode);
	}
	else
	{
		foundPath = false;
	}
}


void PathFinder::createCompletePath(const Vector2f& targetPosition)
{
	const Node* currentNode = &tree.back();
	completePath.clear();
	completePath.push_back(Node(targetPosition));
	while(currentNode->indexPreviousNode != -1)
	{
		completePath.push_back(*currentNode);
		currentNode = &tree[currentNode->indexPreviousNode];
	}
	completePath.push_back(*currentNode);
	std::reverse(completePath.begin(),completePath.end());
}


bool PathFinder::checkForObstaclesNearPosition(const vector<Node>& allObstacles, const Vector2f& position, Vector2f& positionOfObstacle,
		float& smallestDistance)
{
	float currentDistance = parameters.distanceCloseToObstacle;
	bool foundObstacle = false;
	float distance;
	for(vector<Node>::const_iterator i =allObstacles.begin(), end = allObstacles.end(); i!= end; ++i)
	{
		distance = (position - i->position).norm();
		if(distance < currentDistance && i->position != position)
		{
			positionOfObstacle = i->position;
			currentDistance = distance;
			foundObstacle = true;
		}
	}
	smallestDistance = currentDistance;
	return foundObstacle;
}


void PathFinder::addAvoidingPoints(const vector<Node>& allObstacles, vector<Node>& currentUsedTree, const Vector2f& position,
		const Vector2f& positionOfObstacle, bool nearestObstacleMMX, const Node& target, const bool startIsUsed)
{
	int degree = 60;
	Vector2f originalPosition = position;
	float length;
	Vector2f difference;
	if(position == positionOfObstacle)
	{
		length = parameters.distanceToObstacle;
		difference = Vector2f(parameters.distanceToObstacle, 0.0f);
	}
	else
	{
		difference = position - positionOfObstacle;
		difference.normalize(parameters.distanceToObstacle - difference.norm());
		float hypotenuse = (position - positionOfObstacle).norm();
		float opposite = sin(Angle::fromDegrees(degree)) * hypotenuse;
		float adjacent = sqrt(sqr(hypotenuse) - sqr(opposite));
		// length based on "Höhensatz des Euklids"
		length = sqrt((parameters.distanceToObstacle - opposite)	* (parameters.distanceToObstacle + opposite));
		length = length - adjacent;
	}

	const int arraySize = 3;
	float distances[arraySize];
	Node nodes[arraySize];
	bool collisions[arraySize];
	Vector2f obstacles[arraySize];

	nodes[0].position = position + difference;// own direction
	difference.rotate(Angle::fromDegrees(-degree * 2));
	difference.normalize(length);
	nodes[1].position = position + difference;// rotated 70° from original avoiding position, based on start/end position
	difference.rotate(Angle::fromDegrees(-degree * 2));
	nodes[2].position = position + difference;//// rotated -70° from original avoiding position, 180° is not used because obstacle would be between position and new node

	collisions[0] = checkForObstaclesNearPosition(allObstacles, nodes[0].position, obstacles[0],distances[0]); // check for new collisions for all avoiding points
	collisions[1] = checkForObstaclesNearPosition(allObstacles, nodes[1].position, obstacles[1],distances[1]);
	collisions[2] = checkForObstaclesNearPosition(allObstacles, nodes[2].position, obstacles[2],distances[2]);
	//if all avoiding points lead in new obstacles use all points
	if (collisions[0] && collisions[1] && collisions[2] &&
			obstacles[0] != positionOfObstacle && obstacles[1] != positionOfObstacle && obstacles[2] != positionOfObstacle &&
			obstacles[0] != originalPosition && obstacles[1] != originalPosition && obstacles[2] != originalPosition)
	{
		for (int i = 0; i < arraySize; i++)
		{
			currentUsedTree.push_back(nodes[i]);
		}
	}
	else if (startIsUsed)
	{
		Node newPosition;
		float currentDistance = FLT_MAX;
		//use only points which don't lead in another obstacle or where the distance to the new obstacle is high enough.
		//Position is also used if collision exists but only with the avoided obstacle or the own position
		for (int i = 0; i < arraySize; i++)
		{
			if (!collisions[i] || (collisions[i] && (obstacles[i] == positionOfObstacle || obstacles[i] == originalPosition || distances[i] > parameters.distanceToObstacle)))
			{
				float distance = (nodes[i].position - target.position).norm();
				if (distance < currentDistance)
				{
					currentDistance = distance;
					newPosition = nodes[i];
				}
			}
		}
		currentUsedTree.push_back(newPosition);
	}
	else
	{
		// use only points which don't lead in another obstacle or where the distance to the new obstacle is high enough.
		//Position is also used if collision exists but only with the avoided obstacle or the own position
		for (int i = 0; i<arraySize; i++)
		{
			if (!collisions[i] || (collisions[i] && (obstacles[i] == positionOfObstacle|| obstacles[i] == originalPosition|| distances[i] > parameters.distanceCloseToObstacle)))
			{
				currentUsedTree.push_back(nodes[i]);
			}
		}
	}
}

void PathFinder::pathSmooth(const std::vector<Node>& allObstacles){
	unsigned int pathsize = path.size();
	bool inValidFlag = false;
	bool collisionFlag = false;
	Vector2f startOfPath = path[0];
	float distanceToObstacle;
	Vector2f positionObstacle;
	unsigned int nodeNum=0;

//	cout<<"********inner pathSmooth1********"<<endl;
	if(pathsize > 2){
//		cout<<"********inner pathSmooth2********"<<endl;
		for(unsigned int i=1; i<pathsize-1; ++i){
			collisionFlag = checkForCollision(allObstacles, startOfPath,
					path[i],false, distanceToObstacle, positionObstacle);
			if(!collisionFlag){
				inValidFlag = true;
				nodeNum = i;
			}
		}
//		cout<<"********inner pathSmooth3********"<<endl;
		if(inValidFlag){
			for(vector<Vector2f>::iterator iter = path.begin()+1; iter != path.begin()+nodeNum+1; ++iter){
				path.erase(iter);
			}
		}
//		cout<<"********inner pathSmooth4********"<<endl;
	}
}
