#pragma once

#ifdef WIN32
#include <windows.h>

// Nanosecond multimedia timer to measure execution times
class TickTock
{
public:
   void Tick(); // copy the current time to 'tick'
   void Tock(); // copy the current time to 'tock'
   double Elapsed() const; // get difference between 'tick' and 'tock' in milliseconds
private:
   LARGE_INTEGER tick;
   LARGE_INTEGER tock;
};
#endif
