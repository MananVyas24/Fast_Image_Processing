#include "stdafx.h"
#include "Timer.h"

#ifdef WIN32
void TickTock::Tick()
{
   QueryPerformanceCounter(&tick);
}

void TickTock::Tock()
{
   QueryPerformanceCounter(&tock);
}

double TickTock::Elapsed() const
{
   LARGE_INTEGER freq;
   QueryPerformanceFrequency(&freq);
   return static_cast<double>(tock.QuadPart - tick.QuadPart) * 1000.0 / static_cast<double>(freq.QuadPart);
}
#endif
