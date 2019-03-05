#pragma once

#include "opencv2/core.hpp"

struct Bounds
{
	int left;
	int top;
	int right;
	int bottom;

	Bounds() : left(1000000), top(1000000), right(0), bottom(0) { }
	Bounds(int a_left, int a_top, int a_right, int a_bottom) : left(a_left), top(a_top), right(a_right), bottom(a_bottom) { }
	Bounds(const cv::Rect &r) : left(r.x), top(r.y), right(r.x + r.width - 1), bottom(r.y + r.height - 1) { }
	void Extend(const cv::Point& p);
	void Extend(const Bounds& b);
	bool Intersect(const Bounds& b);
	bool Contains(const Bounds& b) const;
	bool Contains(const cv::Point& p) const { return p.x >= left && p.x <= right && p.y >= top && p.y <= bottom; }
	bool Overlaps(const Bounds& other) const { return !(left > other.right || right < other.left) && !(top > other.bottom || bottom < other.top); }
	int Width() const { return right - left + 1; }
	int Height() const { return bottom - top + 1; }
	cv::Point GetCenterPoint() const { return cv::Point((left + right) / 2, (top + bottom) / 2); }
	cv::Rect ToRect() const { return cv::Rect(left, top, Width(), Height()); }
	bool EqualBounds(const Bounds& b) const { return left == b.left && top == b.top && right == b.right && bottom == b.bottom;  };
	int Area() const { return Width() * Height(); }
};

struct SortBoundsByTop
{
	bool operator()(const Bounds& lhs, const Bounds& rhs)
	{
		return lhs.top < rhs.top;
	}
};


bool SegmentsIntersect(cv::Point p1, cv::Point q1, cv::Point p2, cv::Point q2);
bool SegmentsIntersectionPoint(cv::Point p1, cv::Point p2, cv::Point p3, cv::Point p4, cv::Point& intersection);
bool CircleFit(int x1, int y1, int x2, int y2, int x3, int y3, double& x, double& y, double& r);
int CircleFitRadius(int x1, int y1, int x2, int y2, int x3, int y3);

double InternalAngle(double x2, double y2, double x1, double y1, double x3, double y3);
double InternalAngle(const cv::Point& pt2, const cv::Point& pt1, const cv::Point& pt3);
double Angle(double x1, double y1, double x2, double y2);
double Angle(const cv::Point& from, const cv::Point& to);
int AngleQuadrant(double x1, double y1, double x2, double y2);
int Length(int x1, int y1, int x2, int y2);
int Length(const cv::Point& p1, const cv::Point& p2);
double DLength(const cv::Point& p1, const cv::Point& p2);
int LengthSquare(const cv::Point& p1, const cv::Point& p2);
bool AngleLess(double a1, double a2);
bool IsVert(double a1);
bool IsAboutVert(double a1);
bool IsHorz(double a1);
bool IsSlanted(double a1);
double NormAngle(double a);
