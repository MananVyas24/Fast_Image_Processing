#include "stdafx.h"
#include "ConnectedComponents.h"
#include <stdexcept>
#include <algorithm>
#include <assert.h>

BilevelImage ToBilevelImage(const cv::Mat& input, int threshold, bool invert)
{
	//if(input.BitsPerPixel() != 8)
	//	throw std::exception("ToBilevelImage");
	BilevelImage output(input.cols, input.rows);
	uchar px;
	for(int y = 0; y < input.rows; ++y)
	{
		int numBlackRuns = 0;
		int x = 0;
		int from = 0;
		px = input.at<uchar>(y, x);
		for(;;)
		{
			for(;;) // waiting for a black pixel...
			{
				if(!invert && px < threshold || invert && px >= threshold) // black
				{
					from = x;
					//++numBlackRuns;
					++x;
					if(x >= input.cols)
					{
						output.blackRuns.push_back(BlackRun(from, x - 1));
						++numBlackRuns;
						goto EndLine;
					}
					else
					{
						px = input.at<uchar>(y, x);
						break;
					}
				}
				else
				{
					++x;
					if(x >= input.cols)
						goto EndLine;
					else
						px = input.at<uchar>(y, x);
				}
			}
			for(;;) // waiting for a white pixel...
			{
				if(!invert && px >= threshold || invert && px < threshold) // white
				{
					output.blackRuns.push_back(BlackRun(from, x - 1));
					++numBlackRuns;
					++x;
					if(x >= input.cols)
						goto EndLine;
					else
					{
						px = input.at<uchar>(y, x);
						break;
					}
				}
				else
				{
					++x;
					if(x >= input.cols)
					{
						output.blackRuns.push_back(BlackRun(from, x - 1)); // make sure the black lines are always closed
						++numBlackRuns;
						goto EndLine;
					}
					else
						px = input.at<uchar>(y, x);
				}
			}
		}
EndLine:
		output.numBlackRuns.push_back(numBlackRuns);
	}
	return output;
}

/////////////////////////////////////////////////////////////////////////////////////////

/*
PxRect ConnectedComponentShape::Bounds() const
{
	return PxRect(x, y, x + width - 1, y + height - 1);
}

void ConnectedComponentShape::RenderBlacks(Image& output) const
{
	std::vector<BlackRun>::const_iterator it = blackRuns.begin();
	for(int row = y; row < y + height; ++row)
	{
		unsigned char* pRow = output.Row(row);
		for(int rowBlackRuns = numBlackRuns[row - y]; rowBlackRuns > 0 && it != blackRuns.end(); --rowBlackRuns, ++it)
		{
			const BlackRun& blackRun = *it;
			for(int x = blackRun.from; x <= blackRun.to; ++x)
				pRow[x] = 0;
		}
	}
}

bool ConnectedComponentShape::Contains(const PxRect& rect) const
{
	const PxRect componentRect(x, y, x + width - 1, y + height - 1);
	if(rect.bottom < componentRect.top || rect.top > componentRect.bottom || rect.right < componentRect.left || rect.left > componentRect.right)
		return false; // rect is completely outside of this component

	// Now check if there's a black run inside `rect`:
	std::vector<BlackRun>::const_iterator it = blackRuns.begin();
	for(int row = y; row < std::max(componentRect.top, rect.top); ++row)
		it += numBlackRuns[row - y]; // skip rows
	for(int row = std::max(componentRect.top, rect.top); row <= std::min(componentRect.bottom, rect.bottom); ++row)
	{
		for(int rowBlackRuns = numBlackRuns[row - y]; rowBlackRuns > 0 && it != blackRuns.end(); --rowBlackRuns, ++it)
		{
			const BlackRun& blackRun = *it;
			if(!(blackRun.to < rect.left || blackRun.from > rect.right))
				return true; // containment!
		}
	}
	return false;
}
*/

/////////////////////////////////////////////////////////////////////////////////////////

struct Component_
{
	int top;
	int bottom;
	int left;
	int right;
	int y;
	std::vector<uint16_t> numBlackRuns;
	std::vector<BlackRun> blackRuns;

	Component_(int aY, BlackRun blackRun) : top(aY), bottom(aY), left(blackRun.from), right(blackRun.to), y(aY), numBlackRuns(1, 1), blackRuns(1, blackRun) { }

	void Add(int aY, BlackRun blackRun)
	{
		if(aY == y) // existing line
		{
			++numBlackRuns.back();
		}
		else // new line
		{
			assert(aY == y + 1);
			y = aY;
			bottom = aY;
			numBlackRuns.push_back(1);
		}
		blackRuns.push_back(blackRun);
		if(blackRun.from < left)
			left = blackRun.from;
		if(blackRun.to > right)
			right = blackRun.to;
	}

	void MoveTo(Component_& surviving) // move all runs away from here and into the surviving component
	{
		Component_& c1 = *this;
		Component_& c2 = surviving;
		assert(!c1.blackRuns.empty() && !c2.blackRuns.empty());
		std::vector<BlackRun>::const_iterator it1 = c1.blackRuns.begin();
		std::vector<BlackRun>::const_iterator end1 = c1.blackRuns.end();
		std::vector<BlackRun>::const_iterator it2 = c2.blackRuns.begin();
		std::vector<BlackRun>::const_iterator end2 = c2.blackRuns.end();
		std::vector<uint16_t>::const_iterator itNum1 = c1.numBlackRuns.begin();
		std::vector<uint16_t>::const_iterator itNum2 = c2.numBlackRuns.begin();
		Component_ merged(
			std::min(c1.top, c2.top),
			std::max(c1.bottom, c2.bottom),
			std::min(c1.left, c2.left),
			std::max(c1.right, c2.right),
			std::max(c1.y, c2.y));
		merged.blackRuns.reserve(c1.blackRuns.size() + c2.blackRuns.size());
		int n = merged.top;

		while(n < c1.top)
		{
			assert(itNum2 != c2.numBlackRuns.end());
			merged.numBlackRuns.push_back(*itNum2);
			unsigned num = *itNum2++;
			while(num-- > 0)
			{
				assert(it2 != end2);
				merged.blackRuns.push_back(*it2++);
			}
			++n;
		}

		while(n < c2.top)
		{
			assert(itNum1 != c1.numBlackRuns.end());
			merged.numBlackRuns.push_back(*itNum1);
			unsigned num = *itNum1++;
			while(num-- > 0)
			{
				assert(it1 != end1);
				merged.blackRuns.push_back(*it1++);
			}
			++n;
		}

		while(itNum1 != c1.numBlackRuns.end() && itNum2 != c2.numBlackRuns.end())
		{
			//assert(itNum1 != c1.numBlackRuns.end());
			//assert(itNum2 != c2.numBlackRuns.end());
			unsigned num1 = *itNum1;
			unsigned num2 = *itNum2;
			for(;;)
			{
				assert(it1 != end1 && it2 != end2);
				if(it1->from <= it2->from)
				{
					merged.blackRuns.push_back(*it1++);
					if(--num1 == 0)
					{
						while(num2-- > 0)
						{
							assert(it2 != end2);
							merged.blackRuns.push_back(*it2++);
						}
						break;
					}
				}
				else
				{
					merged.blackRuns.push_back(*it2++);
					if(--num2 == 0)
					{
						while(num1-- > 0)
						{
							assert(it1 != end1);
							merged.blackRuns.push_back(*it1++);
						}
						break;
					}
				}
			}
			merged.numBlackRuns.push_back(*itNum1 + *itNum2);
			++itNum1;
			++itNum2;
		}

		while(itNum1 != c1.numBlackRuns.end())
		{
			merged.numBlackRuns.push_back(*itNum1);
			unsigned num = *itNum1++;
			while(num-- > 0)
			{
				assert(it1 != end1);
				merged.blackRuns.push_back(*it1++);
			}
		}

		while(itNum2 != c2.numBlackRuns.end())
		{
			merged.numBlackRuns.push_back(*itNum2);
			unsigned num = *itNum2++;
			while(num-- > 0)
			{
				assert(it2 != end2);
				merged.blackRuns.push_back(*it2++);
			}
		}

		std::vector<uint16_t>().swap(numBlackRuns); // clear
		std::vector<BlackRun>().swap(blackRuns); // clear
		surviving.numBlackRuns.swap(merged.numBlackRuns); // much faster to swap than to assign
		surviving.blackRuns.swap(merged.blackRuns); // much faster to swap than to assign
		surviving.top = merged.top;
		surviving.bottom = merged.bottom;
		surviving.left = merged.left;
		surviving.right = merged.right;
		surviving.y = merged.y;
	/*
		std::vector<unsigned> mergedRunIndexes;
		mergedRunIndexes.reserve(blackRunIndexes.size() + surviving.blackRunIndexes.size());
		assert(!blackRunIndexes.empty() && !surviving.blackRunIndexes.empty());
		std::vector<unsigned>::const_iterator it1 = blackRunIndexes.begin();
		std::vector<unsigned>::const_iterator end1 = blackRunIndexes.begin();
		std::vector<unsigned>::const_iterator it2 = surviving.blackRunIndexes.begin();
		std::vector<unsigned>::const_iterator end2 = surviving.blackRunIndexes.begin();
		for(;;)
		{
			if(*it1 <= *it2)
			{
				mergedRunIndexes.push_back(*it1);
				++it1;
				if(it1 == end1)
				{
					std::copy(it2, end2, std::back_inserter(mergedRunIndexes));
					break;
				}
			}
			else
			{
				mergedRunIndexes.push_back(*it2);
				++it2;
				if(it2 == end2)
				{
					std::copy(it1, end1, std::back_inserter(mergedRunIndexes));
					break;
				}
			}
		}
		blackRunIndexes.clear();
		surviving.blackRunIndexes.swap(mergedRunIndexes); // much faster to swap than to assign
		if(top < surviving.top)
			surviving.top = top;
		//surviving.merged = true;
	*/
	}

	ConnectedComponentShape ToShape() const
	{
		int weight = 0;
		for(std::vector<BlackRun>::const_iterator it = blackRuns.begin(); it != blackRuns.end(); ++it)
			weight += it->to - it->from + 1;
		return ConnectedComponentShape(left, top, right - left + 1, bottom - top + 1, weight/*, numBlackRuns, blackRuns*/);
	}

private:
	Component_(int aTop, int aBottom, int aLeft, int aRight, int aY)
	 : top(aTop), bottom(aBottom), left(aLeft), right(aRight), y(aY) { }
};

static const unsigned NotConnectedIndex = static_cast<unsigned>(-1);

struct BlackRunComponent_
{
	uint16_t from;
	uint16_t to;
	unsigned componentIndex;

	BlackRunComponent_(uint16_t aFrom, uint16_t aTo, unsigned aComponent) : from(aFrom), to(aTo), componentIndex(aComponent) { }

	bool IsConnected() const { return componentIndex != NotConnectedIndex; }
	operator BlackRun() const { return BlackRun(from, to); }
};

std::vector<ConnectedComponentShape> ExtractConnectedComponents(const BilevelImage& input)
{
	assert(!input.numBlackRuns.empty());
	std::vector<Component_> components;
	unsigned componentIndex = 0; // start numbering components from 0
	std::vector<BlackRunComponent_> prevLine; // the previous line, which starts out empty
	int indexImageRun = 0; // start numbering the runs from 0
	std::vector<BlackRun>::const_iterator iterImageRun = input.blackRuns.begin(); // current black run, initialized to first run in input
	int y = 0;
	for(std::vector<uint16_t>::const_iterator iterNBR = input.numBlackRuns.begin(); iterNBR != input.numBlackRuns.end(); ++iterNBR, ++y) // for each line of input
	{
		unsigned numLineRuns = *iterNBR; // number of black runs in the current line
		if(numLineRuns == 0) // skip empty lines
		{
			prevLine.clear(); // the current empty line becomes the previous
		}
		else if(prevLine.empty()) // the previous line is empty
		{
			// there's no need to search for overlaps, every run becomes a new component
			prevLine.reserve(numLineRuns);
			while(numLineRuns-- > 0) // for each run in the current line
			{
				prevLine.push_back(BlackRunComponent_(iterImageRun->from, iterImageRun->to, componentIndex)); // each run starts out as disconnected in the first line
				components.push_back(Component_(y, *iterImageRun));
				++componentIndex;
				++indexImageRun;
				++iterImageRun;
			}
		}
		else
		{
			std::vector<BlackRunComponent_> currLine; // the current line, which will become the next previous
			currLine.reserve(numLineRuns);
			while(numLineRuns-- > 0) // for each run in the current line
			{
				currLine.push_back(BlackRunComponent_(iterImageRun->from, iterImageRun->to, NotConnectedIndex));
				++iterImageRun;
			}

			// match current line runs against previous line runs, searching for overlaps
			assert(!currLine.empty() && !prevLine.empty());
			std::vector<BlackRunComponent_>::iterator iterCurrRun = currLine.begin(); // current line run iterator
			std::vector<BlackRunComponent_>::const_iterator iterPrevRun = prevLine.begin(); // previous line run iterator

			for(;;) // we'll use goto to escape this loop, otherwise the exit conditions would be too complex
			{
				// handle all non-overlapping runs
				do
				{
					while(iterPrevRun->to < iterCurrRun->from - 1)
					{
						++iterPrevRun; // simply skip every previous line run that is left of the current run (clearly no overlap)
						if(iterPrevRun == prevLine.end())
							goto NoMorePrevRuns;
					}

					while(iterPrevRun->from > iterCurrRun->to + 1) // previous line run is on the right of the current run (clearly no overlap)
					{
						if(!iterCurrRun->IsConnected())
						{
							iterCurrRun->componentIndex = componentIndex;
							components.push_back(Component_(y, *iterCurrRun));
							++componentIndex;
						}
						++iterCurrRun; // next current line run
						++indexImageRun;
						if(iterCurrRun == currLine.end())
							goto NoMoreCurrRuns;
					}
				}
				while(iterPrevRun->to < iterCurrRun->from - 1);

				// since we've already handled all non-overlapping components, that means there must be an overlap,
				// which means we need to merge
				if(!iterCurrRun->IsConnected()) // not connected yet
				{
					// this run becomes the same component as the one in the previous line
					iterCurrRun->componentIndex = iterPrevRun->componentIndex;
					components[iterCurrRun->componentIndex].Add(y, *iterCurrRun);
				}
				else if(iterCurrRun->componentIndex != iterPrevRun->componentIndex)
				{
					// There is a conflict, it means the two components must be merged.
					// We are not adding a run to a component, but merging two components with their runs.
					// One of the components will be killed off, while the other one will get all the runs
					// from both. The one we kill gets wiped out, but it is still in the components array,
					// because it would be very time consuming to renumber every index at this time.
					// So it's enough to renumber the dead index to be the surviving one.
					const unsigned deadComponentIndex = iterCurrRun->componentIndex;
					const unsigned survivingComponentIndex = iterPrevRun->componentIndex;
					components[deadComponentIndex].MoveTo(components[survivingComponentIndex]);
					// Now we need to renumber everything in the current and previous lines
					for(std::vector<BlackRunComponent_>::iterator it = currLine.begin(); it != currLine.end(); ++it)
						if(it->componentIndex == deadComponentIndex)
							it->componentIndex = survivingComponentIndex;
					for(std::vector<BlackRunComponent_>::iterator it = prevLine.begin(); it != prevLine.end(); ++it)
						if(it->componentIndex == deadComponentIndex)
							it->componentIndex = survivingComponentIndex;
				}

				// now that we have merged the prev and curr, it's time to move on to the next run to check
				if(iterPrevRun->to < iterCurrRun->to) // the current run reaches beyond the previous one
				{
					++iterPrevRun; // next previous line run
					if(iterPrevRun == prevLine.end())
						goto NoMorePrevRuns;				
				}
				else if(iterPrevRun->to >= iterCurrRun->to) // the previous run reaches beyond the current one
				{
					const int currRunTo = iterCurrRun->to;
					++iterCurrRun; // next current line run
					++indexImageRun;
					if(iterCurrRun == currLine.end())
						goto NoMoreCurrRuns;
					if(iterPrevRun->to == currRunTo)
					{
						++iterPrevRun; // next previous line run
						if(iterPrevRun == prevLine.end())
							goto NoMorePrevRuns;				
					}
				}
			}

NoMorePrevRuns:
			// if the previous line has no more items, we still need to tag all the remaining
			// runs in the current line
			if(iterCurrRun->IsConnected())
				++iterCurrRun;
			while(iterCurrRun != currLine.end())
			{
				iterCurrRun->componentIndex = componentIndex;
				components.push_back(Component_(y, *iterCurrRun));
				++componentIndex;
				++iterCurrRun;
			}

NoMoreCurrRuns:
			// if the current line has no more items, we're done, since whatever is in the
			// previous line, it won't do any difference

			prevLine.swap(currLine); // the current line becomes the previous one
		}
	}

	std::vector<ConnectedComponentShape> result;
	for(std::vector<Component_>::const_iterator itComp = components.begin(); itComp != components.end(); ++itComp)
	{
		const Component_& component = *itComp;
		if(component.blackRuns.empty()) // orphaned component (killed)
			continue;
		result.push_back(component.ToShape());
	}

	return result;
}
