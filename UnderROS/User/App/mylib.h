#ifndef __MYLIB_H
#define __MYLIB_H

#include "usart.h"
#include "ringbuf.h"
#include "tim.h"
#include "SysTick.h"
#include "system.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <ctype.h>
#include <time.h>
#include <math.h>
//#include <conio.h>


#ifndef PI
#define PI (float)3.141592654
#endif

double mapf(double val, double in_min, double in_max, double out_min, double out_max) ;
long map(long val, long in_min, long in_max, long out_min, long out_max);
#endif	/*__MYLIB_H */
