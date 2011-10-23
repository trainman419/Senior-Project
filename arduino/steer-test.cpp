#include "steer.h"
#include <stdio.h>
int main(int argc, char ** argv) {

   // test procedure: verify values by hand; look for correctness at data points
   // and monotonic interpolation between points
   for( int i=-120; i<120; i++ ) {
      printf("% 4d: %f\n", i, steer2radius(i));
   }

   for( float r = -10; r<10; r+=0.01 ) {
      printf("% 2.5f: % 4d\n", r, radius2steer(r));
   }

   return 0;
}
