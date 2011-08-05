#ifndef _POINT_H_
#define _POINT_H_
/* QND N-Dimensional Point class
 * (c) 2011 Christopher "ScribbleJ" Jansen
 *
 */

#include "Globals.h"
#include "config.h"

// Quick and dirty point class
class Point
{
public:
  float axes[NUM_AXES];
  Point() { };
  Point& operator=(const Point& tocopy)
  {
    for(int ax=0;ax<NUM_AXES;ax++)
      axes[ax] = tocopy.axes[ax];
    return *this;
  }
  Point(const Point& tocopy)
  {
    *this = tocopy;
  }
  // No bounds checking
  float& operator[](int idx) { return axes[idx]; }
};


#endif // _POINT_H_
