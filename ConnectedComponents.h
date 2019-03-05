#pragma once
#include "opencv2/core.hpp"
#include <vector>
#include <stdint.h>

//#include <PshPack2.h>
struct BlackRun     // represents a contiguous horizontal black run of pixels
{
	uint16_t from;   // black run's start column (x coordinate)
	uint16_t to;     // black run's end column (x coordinate)
	BlackRun() { }
	BlackRun(uint16_t aFrom, uint16_t aTo) : from(aFrom), to(aTo) { }
};
//#include <PopPack.h>

struct BilevelImage
{
	int width;
	int height;
	std::vector<uint16_t> numBlackRuns;
	std::vector<BlackRun> blackRuns;

	BilevelImage(int aWidth, int aHeight)
	 : width(aWidth), height(aHeight)
	{
		numBlackRuns.reserve(height);
	}
};

BilevelImage ToBilevelImage(const cv::Mat& input, int threshold, bool invert);

struct ConnectedComponentShape
{
	int x, y; // position in the original image
	int width;
	int height;
	int weight; // total number of black (content) pixels
	/*
	std::vector<uint16_t> numBlackRuns;
	std::vector<BlackRun> blackRuns;
	*/

	ConnectedComponentShape(int aX, int aY, int aWidth, int aHeight, int aWeight/*,
		const std::vector<uint16_t>& aNumBlackRuns,
		const std::vector<BlackRun>& aBlackRuns*/)
	 : x(aX), y(aY), width(aWidth), height(aHeight), weight(aWeight)/*, numBlackRuns(aNumBlackRuns), blackRuns(aBlackRuns)*/
	{ }
	/*
	PxRect Bounds() const;
	void RenderBlacks(Image& output) const;
	bool Contains(const PxRect& rect) const;
	*/
};

std::vector<ConnectedComponentShape> ExtractConnectedComponents(const BilevelImage& input);
