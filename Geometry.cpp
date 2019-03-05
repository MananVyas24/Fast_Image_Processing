#include "stdafx.h"
#include "Geometry.h"

#ifndef WIN32 // not Windows
#define __max(a,b) (((a) > (b)) ? (a) : (b))
#define __min(a,b) (((a) < (b)) ? (a) : (b))
#define _countof(a) (sizeof(a)/sizeof(a[0]))
#endif

void Bounds::Extend(const cv::Point& p)
{
	if(p.x < left) left = p.x;
	if(p.y < top) top = p.y;
	if(p.x > right) right = p.x;
	if(p.y > bottom) bottom = p.y;
}

void Bounds::Extend(const Bounds& b)
{
	if(b.left < left) left = b.left;
	if(b.top < top) top = b.top;
	if(b.right > right) right = b.right;
	if(b.bottom > bottom) bottom = b.bottom;
}

bool Bounds::Intersect(const Bounds& b)
{
	if(Overlaps(b) == false)
		return false;

	left = __max(left, b.left);
	top = __max(top, b.top);
	right = __min(right, b.right);
	bottom = __min(bottom, b.bottom);

	return true;
}

bool Bounds::Contains(const Bounds& b) const
{
	Bounds a = Bounds(*this);
	if(!a.Intersect(b))
		return false;

	return a.EqualBounds(b);
}

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
static bool OnSegment(cv::Point p, cv::Point q, cv::Point r)
{
	if(q.x <= __max(p.x, r.x) && q.x >= __min(p.x, r.x) && q.y <= __max(p.y, r.y) && q.y >= __min(p.y, r.y))
		return true;

	return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
static int Orientation(cv::Point p, cv::Point q, cv::Point r)
{
	// See http://www.geeksforgeeks.org/orientation-3-ordered-points/
	// for details of below formula.
	int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

	if(val == 0) return 0;  // colinear

	return (val > 0) ? 1 : 2; // clock or counterclock wise
}

// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
// See http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
// Simpler version: https://ideone.com/fGDEN0
bool SegmentsIntersect(cv::Point p1, cv::Point q1, cv::Point p2, cv::Point q2)
{
	// Find the four orientations needed for general and
	// special cases
	int o1 = Orientation(p1, q1, p2);
	int o2 = Orientation(p1, q1, q2);
	int o3 = Orientation(p2, q2, p1);
	int o4 = Orientation(p2, q2, q1);

	// General case
	if(o1 != o2 && o3 != o4)
		return true;

	// Special Cases
	// p1, q1 and p2 are colinear and p2 lies on segment p1q1
	if(o1 == 0 && OnSegment(p1, p2, q1)) return true;

	// p1, q1 and p2 are colinear and q2 lies on segment p1q1
	if(o2 == 0 && OnSegment(p1, q2, q1)) return true;

	// p2, q2 and p1 are colinear and p1 lies on segment p2q2
	if(o3 == 0 && OnSegment(p2, p1, q2)) return true;

	// p2, q2 and q1 are colinear and q1 lies on segment p2q2
	if(o4 == 0 && OnSegment(p2, q1, q2)) return true;

	return false; // Doesn't fall in any of the above cases
}

// Two lines: 'p1p2' and 'p3p4'
// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
bool SegmentsIntersectionPoint(cv::Point p1, cv::Point p2, cv::Point p3, cv::Point p4, cv::Point& intersection)
{
	const int x1 = p1.x;
	const int y1 = p1.y;
	const int x2 = p2.x;
	const int y2 = p2.y;
	const int x3 = p3.x;
	const int y3 = p3.y;
	const int x4 = p4.x;
	const int y4 = p4.y;
	const int den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
	const int x1y2_y1x2 = x1 * y2 - y1 * x2;
	const int x3y4_y3x4 = x3 * y4 - y3 * x4;
	const int num_x = (x1y2_y1x2) * (x3 - x4) - (x1 - x2) * (x3y4_y3x4);
	const int num_y = (x1y2_y1x2) * (y3 - y4) - (y1 - y2) * (x3y4_y3x4);
	// intersection:
	const int x = cvRound(static_cast<double>(num_x) / den);
	const int y = cvRound(static_cast<double>(num_y) / den);
	// the intersection is not guaranteed to be on the segment, so we need to check it:
	if(__min(x1, x2) <= x && x <= __max(x1, x2) &&
		__min(y1, y2) <= y && y <= __max(y1, y2))
	{
		intersection = cv::Point(x, y);
		return true;
	}
	else
		return false;
}

bool CircleFit(int x1, int y1, int x2, int y2, int x3, int y3, double& x, double& y, double& r)
{
	if(x2 == x1 || x3 == x2)
		return false;
	const double mr = static_cast<double>(y2-y1) / static_cast<double>(x2-x1);
	const double mt = static_cast<double>(y3-y2) / static_cast<double>(x3-x2);
	if(fabs(mr - mt) < 1e-6 || fabs(mr) < 1e-6)
		return false;

	x = (mr * mt * (y3 - y1) + mr * (x2 + x3) - mt * (x1 + x2)) / (2.0 * (mr - mt));
	y = (y1 + y2) / 2.0 - (x - (x1 + x2) / 2.0) / mr;
	r = std::sqrt((x2 - x) * (x2 - x) + (y2 - y) * (y2 - y));
	return true;
}

int CircleFitRadius(int x1, int y1, int x2, int y2, int x3, int y3)
{
	double x, y, r;
	if(!CircleFit(x1, y1, x2, y2, x3, y3, x, y, r))
		return 0;
	else
		return cvRound(r);
}






// http://stackoverflow.com/questions/1211212/how-to-calculate-an-angle-from-three-points
// https://en.wikipedia.org/wiki/Law_of_cosines
// Angle in degrees between two line segments, where x1,y1 is the vertex
// x1,y1      x2,y2
//  o---------o
//   \
//    \
//     o
//    x3,y3
double InternalAngle(double x2, double y2, double x1, double y1, double x3, double y3)
{
	const double L12 = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
	const double L13 = (x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3);
	const double L23 = (x2 - x3) * (x2 - x3) + (y2 - y3) * (y2 - y3);
	const double l12l13 = std::sqrt(L12) * std::sqrt(L13);
	if(fabs(l12l13) < 1e-6)
		return 0.0;
	return acos((L12 + L13 - L23) / (2 * l12l13)) * 180.0 / CV_PI;
}

double InternalAngle(const cv::Point& pt2, const cv::Point& pt1, const cv::Point& pt3)
{
	return InternalAngle(pt2.x, pt2.y, pt1.x, pt1.y, pt3.x, pt3.y);
}

// Absolute angles of line segments in degrees:
//                       270
//           quadrant 3   |    quadrant 4
//                        |
//   180 <----------------+---------------> 0
//                        |
//           quadrant 2   |    quadrant 1
//                       90
// 0,0
//  -----------------------> x
//  |
//  |
// y
double Angle(double x1, double y1, double x2, double y2)
{
	if(fabs(x1 - x2) < 1e-6)
	{
		if(y2 >= y1)
			return 90.0;
		else
			return 270.0;
	}
	else if(x2 > x1)
	{
		if(y2 >= y1)
			return atan((y2 - y1) / (x2 - x1)) * 180.0 / CV_PI;
		else
			return 360.0 + atan((y2 - y1) / (x2 - x1)) * 180.0 / CV_PI;
	}
	else // if(x2 < x1)
	{
		return 180.0 + atan((y2 - y1) / (x2 - x1)) * 180.0 / CV_PI;
	}
}

double Angle(const cv::Point& from, const cv::Point& to)
{
	return Angle(from.x, from.y, to.x, to.y);
}

// Quadrant of absolute angle of line segments (see comment at Angle())
int AngleQuadrant(double x1, double y1, double x2, double y2)
{
	if(x2 > x1)
	{
		if(y2 > y1)
			return 1;
		else if(y2 < y1)
			return 4;
	}
	else if(x2 < x1)
	{
		if(y2 > y1)
			return 2;
		else if(y2 < y1)
			return 3;
	}
	return 0;
}

// The length of a line segment; or the distance between two points
int Length(int x1, int y1, int x2, int y2)
{
	return cvRound(std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
}

int Length(const cv::Point& p1, const cv::Point& p2)
{
	return cvRound(std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)));
}

double DLength(const cv::Point& p1, const cv::Point& p2)
{
	return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

// Same as length, but without the square root. It returns the square of the distance or length. It runs faster than Length().
int LengthSquare(const cv::Point& p1, const cv::Point& p2)
{
	return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

// Compares too absolute angles for directional line segments, in the range of [0 .. 360).
// This function is necessary, because 355 is actually less than 10, since angles rotate around.
// a1, a2: [0 .. 360)
bool AngleLess(double a1, double a2)
{
	if(a1 > 270 && a2 < 180)
		return true; // a1 < a2
	else if(a2 > 270 && a1 < 180)
		return false; // a1 > a2
	else
		return a1 < a2;
}

// Checks if an absolute angle for directional line segments is approximately vertical (90 deg or 270 deg)
// a1: [-180 .. 360)
bool IsVert(double a1)
{
	const bool a1vert = (a1 > 90 - 6 && a1 < 90 + 6) || (a1 > 270 - 6 && a1 < 270 + 6) || (a1 > -90 - 6 && a1 < -90 + 6);
	return a1vert;
}

bool IsAboutVert(double a1)
{
	const bool a1vert = (a1 > 90 - 10 && a1 < 90 + 10) || (a1 > 270 - 10 && a1 < 270 + 10) || (a1 > -90 - 10 && a1 < -90 + 10);
	return a1vert;
}

// Checks if an absolute angle for directional line segments is approximately horizontal (0 deg, 180 deg, 360 deg)
// a1: [0 .. 360)
bool IsHorz(double a1)
{
	const bool a1horz = (a1 > 0 - 6 && a1 < 0 + 6) || (a1 > 180 - 6 && a1 < 180 + 6) || (a1 > 360 - 6 && a1 < 360 + 6);
	return a1horz;
}

// Checks if a angle for directional line segments is slanted like the dominant straight lines in digits "2", "4", "7"
// a1: [0 .. 360)
bool IsSlanted(double a1)
{
	const bool a1slanted = (a1 > 99 && a1 < 135) || (a1 > 279 && a1 < 315) || (a1 >= -81 && a1 <= -45);
	return a1slanted;
}

// Normalizes an angle [0 .. 360) => (-90 .. +90].
// Absolute angles for a directional line segment are in the range of [0 .. 360).
// However, absolute angles for non-directional line segments are in the range of (-90 .. +90], because 270 is like 90, 180 is like 0, etc.
// This function converts a directional angle to a non-directional angle.
double NormAngle(double a)
{
	if(a > 90 && a <= 270)
		return a - 180.0;
	else if(a > 270)
		return a - 360.0;
	else
		return a;
}
