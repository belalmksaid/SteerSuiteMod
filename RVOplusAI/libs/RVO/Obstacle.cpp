//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "RVOSimulator.h"
#include "Obstacle.h"

namespace RVO {
  Obstacle::Obstacle(const Vector2& a, const Vector2& b) {
    _p1 = a;
    _p2 = b;

    _normal = normal(_p1, _p2);
  }

  Obstacle::~Obstacle()
  {
  }

}  // RVO namespace