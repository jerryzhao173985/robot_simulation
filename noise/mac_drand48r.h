#ifndef __MACDRAND48_R_H
#define __MACDRAND48_R_H

#include <stdlib.h>

#include <limits.h>
#ifndef WIN32
#include <mach/mach.h>
#endif

union ieee754_double
  {
    double d;

    /* This is the IEEE 754 double-precision format.  */
    struct
      {
        /* Together these comprise the mantissa.  */
        unsigned int mantissa1:32;
        unsigned int mantissa0:20;
        unsigned int exponent:11;
        unsigned int negative:1;
      } ieee;

    /* This format makes it easier to see if a NaN is a signalling NaN.  */
    struct
      {
        /* Together these comprise the mantissa.  */
        unsigned int mantissa1:32;
        unsigned int mantissa0:19;
        unsigned int quiet_nan:1;
        unsigned int exponent:11;
        unsigned int negative:1;
      } ieee_nan;
  };

#define IEEE754_DOUBLE_BIAS        0x3ff /* Added to exponent.  */


struct drand48_data {
  unsigned short int __x[3];        /* Current state.  */
  unsigned short int __old_x[3]; /* Old state.  */
  unsigned short int __c;        /* Additive const. in congruential formula.  */
  unsigned short int __init;        /* Flag for initializing.  */
  unsigned long long int __a;        /* Factor in congruential formula.  */
};


int srand48_r (long int seedval, struct drand48_data *buffer);

int drand48_r ( struct drand48_data *buffer, double *result);


#endif
