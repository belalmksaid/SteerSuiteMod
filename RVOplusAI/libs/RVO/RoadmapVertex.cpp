//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "RVOSimulator.h"
#include "RoadmapVertex.h"
#include "Obstacle.h"
#include "KDTree.h"

namespace RVO {
  RVOSimulator*  RoadmapVertex::_sim = RVOSimulator::Instance();

  RoadmapVertex::RoadmapVertex(const Vector2& p) 
  {
    _p = p;
  }

  RoadmapVertex::~RoadmapVertex()
  {
  }

  void RoadmapVertex::computeNeighbors(float radius)
  {
    _neighbors.clear();
    for (int i = 0; i < (int) _sim->_roadmapVertices.size(); ++i) {
      if (_sim->_roadmapVertices[i] != this && _sim->_kdTree->queryVisibility(_p, _sim->_roadmapVertices[i]->_p, radius)) {
        addNeighbor(abs(_sim->_roadmapVertices[i]->_p - _p), i);
      }
    }
  }

  void RoadmapVertex::addNeighbor(float distance, int neighbor_id) {
    _neighbors.push_back(std::make_pair(distance, neighbor_id));
  }
}