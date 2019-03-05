#include "stdafx.h"
#include "Contour.h"
#include "Geometry.h"

#ifndef WIN32 // not Windows
#define __max(a,b) (((a) > (b)) ? (a) : (b))
#define __min(a,b) (((a) < (b)) ? (a) : (b))
#define _countof(a) (sizeof(a)/sizeof(a[0]))
#endif

#ifdef CANNY
// When calling cv::findContours on a canny image, the countour could all the way around and then back, snaking around and around.
// This function breaks up a contour when it turns around (i.e. when the same points are visited again). Breaking up a closed contour
// will make it an open contour (or simply a line).
// imageRows, imageCols: the size of the canny image.
void NormalizeContours(std::vector<Contour>& contours, int imageRows, int imageCols)
{
	for(size_t n = 0; n < contours.size(); ++n)
	{
		std::vector<cv::Point>& contourPoints = contours[n].points;
		cv::Mat visited = cv::Mat::zeros(imageRows, imageCols, CV_8U); // 0 means the pixel is not visited yet; 1 means it was visited before
		for(std::vector<cv::Point>::iterator it = contourPoints.begin(); it != contourPoints.end(); ++it)
		{
			uchar& pix = visited.at<uchar>(it->y, it->x);
			if(pix == 0)
				pix = 1;
			else
			{
				// split
				Contour split(it, contourPoints.end(), false, contours[n].originalContourIndex);
				contourPoints.erase(it, contourPoints.end());
				contours[n].closed = false;
				if(n == contours.size() - 1)
					contours.push_back(split);
				else
					contours.insert(contours.begin() + n + 1, split);
				break;
			}
		}
	}
}
#endif

// Find open contours whose end points are very close to each other. That suggests those contours are considered connected.
// For example, if a line has a small gap, they really belong together.
// Returns a map that binds original contour indexes that are close to each other.
std::map<size_t, std::set<size_t> > FindNearbyOpenContours(const std::vector<Contour>& contours, int maxDistance)
{
	const int maxDistanceSquare = maxDistance * maxDistance;
	std::map<size_t, std::set<size_t> > nearbyMap;
	if(contours.empty())
		return nearbyMap;
	for(size_t i = 0; i < contours.size() - 1; ++i)
	{
		for(size_t j = i + 1; j < contours.size(); ++j)
		{
			if(contours[i].originalContourIndex == contours[j].originalContourIndex)
				continue;
			const cv::Point& point_i_first = contours[i].points.front();
			const cv::Point& point_i_last = contours[i].points.back();
			const cv::Point& point_j_first = contours[j].points.front();
			const cv::Point& point_j_last = contours[j].points.back();
			if(LengthSquare(point_i_first, point_j_first) < maxDistanceSquare ||
				LengthSquare(point_i_first, point_j_last) < maxDistanceSquare ||
				LengthSquare(point_i_last, point_j_first) < maxDistanceSquare ||
				LengthSquare(point_i_last, point_j_last) < maxDistanceSquare)
			{
				nearbyMap[contours[i].originalContourIndex].insert(contours[j].originalContourIndex);
				nearbyMap[contours[j].originalContourIndex].insert(contours[i].originalContourIndex);
			}
		}
	}
	return nearbyMap;
}

// nearbyMap is a tree of nearby contours that are considered connected. But this data structure
// is too complicated, it needs to be simplified. This function maps contour indexes into one
// common index, which is then used to address the entire group of connected contours.
// Contours that aren't connected will be missing from the result.
// For example, if contours 0, 2, 5 are connected, they will all be substituted by the smallest
// index, which is 0. The result is going to contain:
// 2 -> 0
// 5 -> 0
// The algorithm is based on tree traversal.
// Example: nearbyMap = { 0 -> { 2 },   2 -> {0, 5},   5 -> { 2 } };
// result: { 2 -> 0, 5 -> 0 }
std::map<size_t, size_t> SimplifyNearbyMap(const std::map<size_t, std::set<size_t> >& nearbyMap)
{
	std::map<size_t, size_t> result;
	std::set<size_t> visited; // do not repeatedly traverse the same node
	for(std::map<size_t, std::set<size_t> >::const_iterator it = nearbyMap.begin(); it != nearbyMap.end(); ++it)
	{
		const size_t currentKey = it->first;
		if(!visited.insert(currentKey).second) // we've already dealt with this one before
			continue;

		const std::set<size_t>& currentValues = it->second;
		std::set<size_t> group; // these contours are in the same group
		std::set<size_t> toVisit; // stack for tree traversal: we still have to visit these ones

		// visit the currentKey:
		group.insert(currentKey);
		for(std::set<size_t>::const_iterator it = currentValues.begin(); it != currentValues.end(); ++it)
		{
			if(visited.find(*it) == visited.end()) // we've not visited this before
			{
				toVisit.insert(*it);
				group.insert(*it);
			}
		}

		// traverse the tree:
		while(!toVisit.empty())
		{
			const size_t visitingKey = *toVisit.begin(); // fetch from stack: visit this next
			toVisit.erase(toVisit.begin()); // remove it from the stack
			if(visited.insert(visitingKey).second) // we've not dealt with this one before
			{
				std::map<size_t, std::set<size_t> >::const_iterator found = nearbyMap.find(visitingKey);
				assert(found != nearbyMap.end());
				if(found != nearbyMap.end())
				{
					const std::set<size_t>& visitingValues = found->second;
					for(std::set<size_t>::const_iterator it = visitingValues.begin(); it != visitingValues.end(); ++it)
					{
						if(visited.find(*it) == visited.end()) // we've not visited this before
						{
							toVisit.insert(*it);
							group.insert(*it);
						}
					}
				}
			}
		}

		// now we have a group of contours that we consider connected
		if(!group.empty())
		{
			std::set<size_t>::const_iterator it = group.begin();
			const size_t groupIndex = *it; // pick the smallest contour index as the group index
			// Map each contour index in the group into groupIndex:
			for(++it; it != group.end(); ++it)
				result.insert(std::make_pair(*it, groupIndex));
		}
	}

	return result;
}
