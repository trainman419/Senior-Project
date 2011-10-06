#include "SimpleGPS.hpp"

int main(int argc, char ** argv) {
   char buffer[sizeof(SimpleGPS)];
   SimpleGPS f(buffer);
   f.latitude = 25;
   f.longitude = -294;
   long foo = f.latitude;
   return foo;
}
