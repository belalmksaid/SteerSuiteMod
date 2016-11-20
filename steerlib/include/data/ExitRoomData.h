#ifndef __EXIT_ROOM_DATA_H__
#define __EXIT_ROOM_DATA_H__

#include "Globals.h"
#include "util/Geometry.h"

using namespace std;
using namespace Util;

inline bool onSegment(Point p, Point q, Point r)
{
	if (q.z <= max(p.z, r.z) && q.z >= min(p.z, r.z) &&
		q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x))
		return true;
	return false;
}

inline int orientation(Util::Point p, Util::Point q, Util::Point r)
{
	int val = (q.x - p.x) * (r.z - q.z) -
		(q.z - p.z) * (r.x - q.x);

	if (val == 0) return 0;  // colinear
	return (val > 0) ? 1 : 2; // clock or counterclock wise
}

inline bool doIntersect(Util::Point p1, Util::Point q1, Util::Point p2, Util::Point q2)
{
	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// General case
	if (o1 != o2 && o3 != o4)
		return true;

	// Special Cases
	// p1, q1 and p2 are colinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(p1, p2, q1)) return true;

	// p1, q1 and p2 are colinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;

	// p2, q2 and p1 are colinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;

	// p2, q2 and q1 are colinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	return false; // Doesn't fall in any of the above cases
}

inline bool isInside(vector<Util::Point> polygon, int n, Util::Point p)
{
	// There must be at least 3 vertices in polygon[]
	if (n < 3)  return false;

	// Create a point for line segment from p to infinite
	Util::Point extreme(p.x, 0, 10000.0f);

	// Count intersections of the above line with sides of polygon
	int count = 0, i = 0;
	do
	{
		int next = (i + 1) % n;

		// Check if the line segment from 'p' to 'extreme' intersects
		// with the line segment from 'polygon[i]' to 'polygon[next]'
		if (doIntersect(polygon[i], polygon[next], p, extreme))
		{
			// If the point 'p' is colinear with line segment 'i-next',
			// then check if it lies on segment. If it lies, return true,
			// otherwise false
			if (orientation(polygon[i], p, polygon[next]) == 0)
				return onSegment(polygon[i], p, polygon[next]);

			count++;
		}
		i = next;
	} while (i != 0);

	// Return true if count is odd, false otherwise
	return count & 1;  // Same as (count%2 == 1)
}

inline double angle2D(double x1, double y1, double x2, double y2)
{
	double dtheta, theta1, theta2;

	theta1 = atan2(y1, x1);
	theta2 = atan2(y2, x2);
	dtheta = theta2 - theta1;
	while (dtheta > M_PI)
		dtheta -= M_2_PI;
	while (dtheta < -M_PI)
		dtheta += M_2_PI;

	return(dtheta);
}

inline bool insidePolygon(vector<Point> polygon, Point p)
{
	int n = polygon.size();
	int i;
	double angle = 0;
	Point p1, p2;

	for (i = 0;i<n;i++) {
		cout << polygon[i] << endl;
		p1.x = polygon[i].x - p.x;
		p1.z = polygon[i].z - p.z;
		p2.x = polygon[(i + 1) % n].x - p.x;
		p2.z = polygon[(i + 1) % n].z - p.z;
		angle += angle2D(p1.x, p1.z, p2.x, p2.z);
	}

	if (abs(angle) < M_PI)
		return false;
	else
		return true;
}


#ifdef _WIN32
// on win32, there is an unfortunate conflict between exporting symbols for a
// dynamic/shared library and STL code.  A good document describing the problem
// in detail is http://www.unknownroad.com/rtfm/VisualStudio/warningC4251.html
// the "least evil" solution is just to simply ignore this warning.
#pragma warning( push )
#pragma warning( disable : 4251 )
#endif

namespace SteerLib {

	class STEERLIB_API Room;

	class STEERLIB_API Exit {
	public:
		string number;
		Point position;
		Vector normal;
		vector<Room*> rooms;
		float weight1, weight2;

		Exit() { weight1 = weight2 = -1; }
	};

	class STEERLIB_API Room {
	public:
		string number;
		Point position;
		float area;
		vector<Exit*> exits;
		vector<Point> vertices;
		bool containsAgent(Point pos);
	};

	class STEERLIB_API EvacuationTarget {
	public:
		Point position;
	};

}

#endif