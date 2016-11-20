//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

/* \file RoadmapVertex.h Contains the class RoadmapVertex. */

#ifndef __ROADMAP_VERTEX_H__
#define __ROADMAP_VERTEX_H__

#include "RVODef.h"

namespace RVO {

  /* The class defining a roadmap vertex. */
  class RoadmapVertex
  {
  private:
    /* Constructor. Constructs a roadmap vertex.
      \param p The position of the roadmap vertex. */
    RoadmapVertex(const Vector2& p);
    /* Deconstructor. */
    ~RoadmapVertex();

    /* Adds all visible roadmap vertices for the specified radius to the list of neighbors.
    */
    void computeNeighbors(float radius);
    /* Adds a roadmap vertex to the list of neighbors.
      \param distance The distance to the neighboring roadmap vertex (used for shortest path planning).
      \param neighbor_id The ID of the neighboring roadmap vertex. */
    void addNeighbor(float distance, int neighbor_id);

    /* The position of the roadmap vertex */
    Vector2 _p;

    /* The list of neighbors of the roadmap vertex. The vector contains pairs of distance to the neighbor and roadmap vertex ID of the neighbor. */
    std::vector<std::pair<float, int> > _neighbors;  // list of neighbors (distance to neighboring vertex, index of neighbor vertex)

  protected:
    /* A reference to the singleton simulator. */
    static RVOSimulator* _sim;

    friend class Agent;
    friend class Roadmap;
    friend class Goal;
    friend class RVOSimulator;
  };
}
#endif
