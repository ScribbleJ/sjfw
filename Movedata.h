#ifndef _MOVEDATA_H_
#define _MOVEDATA_H_


struct Movedata
{
  enum { READY, RUNNING, DONE } state;
  unsigned long nominal_movetime;
  unsigned long startinterval;

};

#endif // _MOVEDATA_H_
