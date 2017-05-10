#pragma once
#include "Tools/Streams/AutoStreamable.h"
STREAMABLE(PenaltyShootParameter,
{,
 (float) readyLocationX,
 (float) readyLocationY, (float) readyLocationR, (float) readyDeltaY,
 (float) readyDeltaR, (float) speedX, (float) speedY, (float) speedR,
 (float) ballDangerous, (float) saveDirectionFromBall,
 (int) ballNotSeen, (int) keeperDeadTime, (int) offsetLeft,
 (int) offsetRight, (int) direction,
});
