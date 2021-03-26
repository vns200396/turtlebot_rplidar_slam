#include "mylib.h"


double mapf(double val, double in_min, double in_max, double out_min, double out_max) 
{
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


long map(long val, long in_min, long in_max, long out_min, long out_max)
{
  // if input is smaller/bigger than expected return the min/max out ranges value
  if (val < in_min)
    return out_min;
  else if (val > in_max)
    return out_max;

  // map the input to the output range.
  // round up if mapping bigger ranges to smaller ranges
  else  if ((in_max - in_min) > (out_max - out_min))
    return (val - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
  // round down if mapping smaller ranges to bigger ranges
  else
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
