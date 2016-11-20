#include "data/ExitRoomData.h"
using namespace SteerLib;

bool Room::containsAgent(Point pos) {
	return isInside(vertices, vertices.size(), pos);
}