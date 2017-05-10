
#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct BallSpot
 * A struct that represents a spot that might be an indication of a ball.
 */
STREAMABLE(BallSpot,
           {
    BallSpot() = default;
    BallSpot(Vector2i pos) : position(pos){};
    BallSpot(int x, int y) : position(x, y){}, 
	(Vector2i)position, 
	(float)shouldR, 
	(float)radiusInImage, 
	(float)distance, 
	(int)colorH, 
	(int)colorS, 
	(int)colorY, 
	(int)y, 
	(int)cb, 
	(int)cr, 
	(bool)found, 
	(bool)centerFound,
});