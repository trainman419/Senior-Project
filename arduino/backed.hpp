/* crazy */

#include <stdlib.h>
#include <stdint.h>

template <typename T> class backed_T {
   private:
      char * backing;
      T foo;
   public:
      backed_T(char * b) : backing(b) {}

      backed_T & operator=(const T & other) {
         for(size_t i=0; i<sizeof(T); i++ ) {
            backing[i] = (other >> (i*8)) & 0xFF;
         }
         return *this;
      }

      operator T() {
         T ret;
         for(size_t i=0; i<sizeof(T); i++ ) {
            ret |= backing[i] << (i*8);
         }
         return ret;
      }
};

class SimpleGPS {
   public:
      SimpleGPS(char * b) : 
         latitude(b), 
         longitude(b + sizeof(long)), 
         altitude(b + sizeof(long) + sizeof(long)),
         date(b + sizeof(long) + sizeof(long) + sizeof(long)),
         time(b + sizeof(long) + sizeof(long) + sizeof(long) + sizeof(unsigned long)),
         fix_age(b + sizeof(long) + sizeof(long) + sizeof(long) + sizeof(unsigned long) + sizeof(unsigned long)) {}


      backed_T<long> latitude;
      backed_T<long> longitude;
      backed_T<long> altitude;
      backed_T<unsigned long> date;
      backed_T<unsigned long> time;
      backed_T<unsigned long> fix_age;

};
