//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

/* \file RVODef.h Defines global functions, constants and includes used throughout the solution. */

#ifndef __RVO_DEF_H__
#define __RVO_DEF_H__

/* Pi. */
#define RVO_PI 3.14159265358979323846f

/* Epsilon. */
#define RVO_EPSILON 0.00001f

#include <map>
#include <vector>
#include <algorithm>
#include <time.h>
#include "vector2.h"

namespace RVO {
  // forward declarations
  class Obstacle;
  class RoadmapVertex;
  class Agent;
  class RVOSimulator;

  // global functions

  /* Square.
     \param a A scalar
     \returns Returns the square of the scalar.
    */
  inline float sqr(float a) {
    return a*a;
  }

  /* Signed distance between point and line.
     \param a The first point of the line
     \param b The second point of the line
     \param c The point
     \returns Returns the signed distance times the length of segment ab between line ab (not the segment) and point c. Is positive when c lies to the left of ab.
    */
  inline float leftOf(const Vector2& a, const Vector2& b, const Vector2& c) {
    return det(a - c, b - a);
  }

  /* Time to collision of a ray to a line segment.
     \param p The start position of the ray
     \param v The velocity vector of the ray
     \param a The first endpoint of the line segment
     \param b The second endpoint of the line segment
     \param collision Specifies whether the time to collision is computed (false), or the time from collision (true).
     \returns Returns the time to collision of ray p + tv to segment ab, and #RVO_INFTY when the line segment is not hit. If collision is true, the value is negative.
    */
  inline float timeToCollision(const Vector2& p, const Vector2& v, const Vector2& a, const Vector2& b, bool collision) {
    float D = det(v, b - a);
    if (D == 0) {  // ray and line are collinear
      if (collision) {
        return -RVO_INFTY;
      } else {
        return RVO_INFTY;
      }
    }

    float invD = 1.0f/D;
    float t = det(a - p, b - a) * invD;
    float s = det(p - a, v) * -invD;

    if (t < 0 || s < 0 || s > 1) {
      if (collision) {
        return -RVO_INFTY;
      } else {
        return RVO_INFTY;
      }
    } else {
      return t;
    }
  }

  /* Time to collision of a ray to a disc.
     \param p The start position of the ray
     \param v The velocity vector of the ray
     \param p2 The center position of the disc
     \param radius The radius of the disc
     \param collision Specifies whether the time to collision is computed (false), or the time from collision (true).
     \returns Returns the time to collision of ray p + tv to disc D(p2, radius), and #RVO_INFTY when the disc is not hit. If collision is true, the value is negative.
    */
  inline float timeToCollision(const Vector2& p, const Vector2& v, const Vector2& p2, float radius, bool collision) {
    Vector2 ba = p2 - p;
    float sq_diam = radius * radius;
    float time;

    float discr = -sqr(det(v, ba)) + sq_diam * absSq(v);
    if (discr > 0) {
      if (collision) {
        time = (v * ba + std::sqrt(discr)) / absSq(v);
        if (time < 0) {
          time = -RVO_INFTY;
        }
      } else {
        time = (v * ba - std::sqrt(discr)) / absSq(v);
        if (time < 0) {
          time = RVO_INFTY;
        }
      }
    } else {
      if (collision) {
        time = -RVO_INFTY;
      } else {
        time = RVO_INFTY;
      }
    }
    return time;
  }

  /* Squared distance of a point to a line segment.
     \param a The first endpoint of the line segment
     \param b The second endpoint of the line segment
     \param c The point
     \returns Returns the squared distance between line segment ab and point c.
    */
  inline float distSqPointLineSegment(const Vector2& a, const Vector2& b, const Vector2& c) {
    float r = ((c - a)*(b - a)) / absSq(b - a);

    if (r < 0) { // point a is closest to c
      return absSq(c - a);
    } else if (r > 1) { // point b is closest to c
      return absSq(c - b);
    } else { // some point in between a and b is closest to c
      return absSq(c - (a + r*(b - a)));
    }
  }

}

#endif
