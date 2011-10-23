/*
 * Implementation of functions from steer.h; conversions from steering setting
 * to turning radius and back
 */

#include "steer.h"

#include <math.h>


// TODO: measure an appropriate value for radius[0]

// radius table for steering settings in increments of 10
// steer:              0     10,    20,    30,    40,    50,    60,    70,
float radius[] = { 100.0, 5.550, 3.875, 2.900, 1.960, 1.560, 1.290, 1.025,
//    80,    90,   100
   0.905, 0.790, 0.695 };

int8_t radius2steer(float r) {
   r = r<0?-r:r; // abs(r)
   int8_t i = 1;
   // find where this radius falls in our table
   for( i=1; i<11 && r<radius[i]; ++i );
   // radius[i-1] > r > radius[i]
   float base = radius[i];
   float inc = (radius[i-1] - base) / 10.0;
   float diff = r - base; // diff may be negative if i==10
   int8_t s = i*10 - round(diff / inc);
   return s;
}

float steer2radius(int8_t s) {
   s = s<0?-s:s; // abs(s)
   int8_t i = s / 10;
   if( i > 10 ) i = 10;
   float base = radius[i];
   // if we aren't at a measured value, estimate
   if( s - i*10 != 0 ) {
      int8_t diff = s - i*10;
      float inc;
      // if we're within our measured curve, interpolate
      if( i < 10 ) {
         inc = (base - radius[i+1]) / 10.0;
      } else {
         // if we're above our measured curve, extrapolate from top two points
         inc = (radius[i-1] - base) / 10.0;
      }
      base -= inc * diff;
   }
   return base;
}
