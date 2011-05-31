#ifndef _MOVEDATA_H_
#define _MOVEDATA_H_

#include "config.h"
#include "Point.h"

struct Movedata
{
public:
  float feed;

  unsigned long movesteps;
  unsigned long axismovesteps[NUM_AXES];
  bool          axisdirs[NUM_AXES];
  int           leading_axis;

  unsigned long startinterval;
  unsigned long fullinterval;
  unsigned long steps_to_accel;
  unsigned long accel_until;
  unsigned long decel_from;
  unsigned long accel_inc;

  Point         endpos;
  Point         startpos;
};

#endif // _MOVEDATA_H_
