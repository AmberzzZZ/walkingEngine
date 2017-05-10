/**
* @file LibWalk.h
* @author Xiangwei Wang
* @date March 18th, 2014 
*/

class LibWalk: public LibraryBase
{
public:
	LibWalk();
	
	void preProcess() override;
	void postProcess() override;

	bool toAvoid; //flag on whether to avoid obstacle by moving to left or right
	bool avoidToLeft; // true for moving left to avoid, false for moving right
	PathFinder thePathFinder;

	/**
	 * @function to find the path to go avoiding the obstacle
	 */
//	Vector2f findPath(Vector2f startLocation, Vector2f targetLocation);

	/**
	 * @function to find the path start where robot is, to go avoiding the obstacle
	 */
	Vector2f findPath(Vector2f targetLocation);
	
	/**
	 * @function to find the path where robot is, end with the ball location to go avoiding the abstacle
	 */
//	Vector2f findPath();

	/**
	 * @function whether robot contact left
	 * @return true for contacting left, false for not
	 */
	bool isContactLeft();

	/**
	 * @function whether robot contact right
	 * @return true for contacting right, false for not
	 */
	bool isContactRight();

	/**
	 *
	 */
	Vector2f findNextTarget(Vector2f finalTarget);

	Vector2f needAvoidObstcale(Vector2f start, Vector2f end, Vector2f obs);
};
