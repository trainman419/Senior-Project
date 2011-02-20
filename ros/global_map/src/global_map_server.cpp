/* global_map_server.cpp
 *
 * A map server to operate in a global context
 *
 * Based on a transverse mercator projection, with a separate map for each 
 *  1 degree latitude, each map being 2 degrees wide
 *
 * Author: Austin Hendrix
 */

/* Future thoughts:
 *
 * ability to load and save maps to disk?
 * ability to merge data from separate meridians when traversing from one 
 *  meridian to another
 */

#include <map>
#include "ros/ros.h"
#include "global_map/GlobalMap.h"

// smallest chunk of map
//  defined to be 1MB each
#define HUNK_SZ 1024
typedef uint8_t map_hunk[HUNK_SZ][HUNK_SZ];

// the in-memory map cache
typedef map<pair<int16_t, int16_t>, map_hunk* > cache_type;

int main(int argc, char ** argv, char ** envp) {
   return 0;
}
