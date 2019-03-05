#pragma once
#include "opencv2/core.hpp"
#include <vector>
#include <set>
#include <map>

struct Contour
{
	std::vector<cv::Point> points;
	bool closed;
	size_t originalContourIndex;

	Contour(const std::vector<cv::Point>& a_points, bool a_closed, size_t a_originalContourIndex)
	 : points(a_points), closed(a_closed), originalContourIndex(a_originalContourIndex)
	{
	}

	Contour(std::vector<cv::Point>::const_iterator begin, std::vector<cv::Point>::const_iterator end, bool a_closed, size_t a_originalContourIndex)
	 : points(begin, end), closed(a_closed), originalContourIndex(a_originalContourIndex)
	{
	}
};

#ifdef CANNY
void NormalizeContours(std::vector<Contour>& contours, int imageRows, int imageCols);
#endif

std::map<size_t, std::set<size_t> > FindNearbyOpenContours(const std::vector<Contour>& contours, int maxDistance);
std::map<size_t, size_t> SimplifyNearbyMap(const std::map<size_t, std::set<size_t> >& nearbyMap);
