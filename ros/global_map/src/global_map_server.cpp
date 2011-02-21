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
#include <string>
#include <stdint.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>

// ROS includes
#include "ros/ros.h"
#include "ros/assert.h"
#include "global_map/Map.h"
#include "global_map/Update.h"
#include "global_map/GetMeridian.h"
#include "global_map/SetMeridian.h"
#include "global_map/Offset.h"

using namespace std;

// smallest chunk of map
//  defined to be 1MB each
#define HUNK_SIDE 1024
#define HUNK_SZ (HUNK_SIDE * HUNK_SIDE)
class map_hunk {
public:
   int last_use;
   int8_t * data;

   map_hunk() : last_use(0) { 
      data = (int8_t*)malloc(HUNK_SZ);
      if( data == NULL ) {
         ROS_ERROR("Failed to allocate map hunk: %s", strerror(errno));
      }
   }
   ~map_hunk() { free(data); }
};
//typedef uint8_t map_hunk[HUNK_SZ][HUNK_SZ];

// the in-memory map cache
typedef pair<int16_t, int16_t> hunk_idx;
typedef map<hunk_idx, map_hunk > cache_type;
cache_type * cache;

// maximum number of entries to have in the cache at a time. 64MB
#define MAX_CACHE _SZ 64

// usage counter; for updating last-used counts
int use_count = 0;

// Global Meridian
int16_t meridian;

// the path where we store our maps
string map_path = "/tmp/global_map";

/*****************************************************************************/
/* End definitions. Begin code */

// get the meridian
bool getMeridian(global_map::GetMeridian::Request &request,
                 global_map::GetMeridian::Response &response) {
   response.meridian = meridian;
   return true;
}

// set the meridian
//  TODO: convert all current data to new meridian space.
//  TODO: flush current cache to disk and clear cache
//  TODO: ensure that paths for new and old on-disk locations exist
bool setMeridian(global_map::SetMeridian::Request &req,
                 global_map::SetMeridian::Response & resp) {
   meridian = req.meridian;
   return true;
}

// compute row/column offsets for lat/lon from current meridian
// see: http://en.wikipedia.org/wiki/Transverse_Mercator_projection#Formulae_for_the_spherical_Transverse_Mercator
const static double EARTH_RADIUS = 6378.1; // kilometers
const static double A = 1.0;

// row (or Y)
int16_t row(double lon) {
   double lon_tmp = lon - meridian;
   // we should never be more than 1 degree from the meridian
   ROS_ASSERT(lon_tmp < 1.0);
   ROS_ASSERT(lon_tmp > -1.0);

   double y = A / 2 * log( (1 + sin(lon_tmp)) / (1 - sin(lon_tmp)) );
   cout << "lon: " << lon << " y: " << y << endl;
   return y; // implicit cast on return
}

// col (or X)
int16_t col(double lat) {
   //double lon_tmp = lon - meridian;
   double x = -A * lat;
   cout << "lat: " << lat << " x: " << x << endl;
   return x; // implicit cast on return
}

// Service to get offset from meridian center to specific lat/lon
bool getOffset(global_map::Offset::Request &req,
               global_map::Offset::Response &resp) {
   resp.row = row(req.lon);
   resp.col = col(req.lat);
   return true;
}

// functions to maintain the in-memory map cache
inline int use() {
   return use_count++;
}

// get the path to a map hunk. caller is responsible for freeing buffer
char * hunk_path(hunk_idx idx) {
   char * path = (char*)malloc(1024);
   int cnt = snprintf(path, 1024, "%s/%d/%d-%d.map", map_path.c_str(),
                      meridian, idx.first, idx.second);
   if( cnt >= 1024 ) {
      free(path);
      path = (char*)malloc(cnt+1);
      cnt = snprintf(path, cnt+1, "%s/%d/%d-%d.map", map_path.c_str(), 
                      meridian, idx.first, idx.second);
   }
   return path;
}

// load the hunk at a particular hunk row/column address
void load_hunk(hunk_idx idx) {
   char * path = hunk_path(idx);

   cache_type::iterator hunk_itr = cache->find(idx);

   // we should never be trying to load a hunk that already exists
   ROS_ASSERT(hunk_itr == cache->end());

   map_hunk hunk;

   int in = open(path, O_RDONLY);
   if( in < 0 ) {
      // failed to open file. complain and initialize to 0
      ROS_WARN("Failed to open map hunk(initializing to unknown) %s: %s", 
            path, strerror(errno));
      for( int i=0; i < HUNK_SZ; i++ ) {
         hunk.data[i] = -1;
      }
   } else {
      int cnt = read(in, hunk.data, HUNK_SZ);
      if( cnt != HUNK_SZ ) {
         ROS_ERROR("Hunk read error; only read %d bytes for hunk %s; expected %d", cnt, path, HUNK_SZ);
         for( int i = cnt; i < HUNK_SZ; i++ ) {
            hunk.data[i] = -1;
         }
      }
      close(in);
   }

   // insert new hunk into the cache
   (*cache)[idx] = hunk;

   free(path);
}

void save_hunk(hunk_idx idx) {
   char * path = hunk_path(idx);

   cache_type::iterator hunk = cache->find(idx);

   // we should never be trying to save a hunk that doesn't exist
   ROS_ASSERT(hunk != cache->end());

   // open file for output
   int out = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
   if( out < 0 ) {
      ROS_ERROR("Error opening: %s: %s", path, strerror(errno));
   } else {
      int cnt = write(out, hunk->second.data, HUNK_SZ);
      if( cnt != HUNK_SZ ) {
         ROS_ERROR("Problem writing to %s: %s", path, strerror(errno));
      }
      close(out);
   }

   free(path);
   return;
}

int main(int argc, char ** argv, char ** envp) {

   ros::init(argc, argv, "global_map_server");
   ros::NodeHandle n;


   // TODO: initialize map path from configuration variable

   cache = new cache_type();
   meridian = 0; // Greenwich

   ros::ServiceServer get_m_serv = n.advertiseService("GetMeridian",
                                                      getMeridian);
   ros::ServiceServer set_m_serv = n.advertiseService("SetMeridian",
                                                      setMeridian);
   ros::ServiceServer offset_serv = n.advertiseService("Offset", getOffset);

   return 0;
}
