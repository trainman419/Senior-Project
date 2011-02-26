/* test_offset.cpp
 *
 * A program to test the offset call; checking that conversion from 
 * lat/lon to grid indices is sane and proper.
 *
 * Author: Austin Hendrix
 */

#include <errno.h>
#include <stdio.h>
#include "ros/ros.h"
#include "global_map/Offset.h"
#include "global_map/SetMeridian.h"

int main(int argc, char ** argv) {
   
   ros::init(argc, argv, "test_offset");
   if( argc != 2 ) {
      ROS_ERROR("Usage: test_offset <file>");
      return -1;
   }

   // open file for reading
   FILE *  infile = fopen(argv[1], "r");
   if( infile == NULL ) {
      ROS_ERROR("Problem opening %s: %s", argv[1], strerror(errno));
      return -1;
   }

   double lat, lon;

   ros::NodeHandle n;
   ros::ServiceClient o_client = n.serviceClient<global_map::Offset>("Offset");
   global_map::Offset o_srv;

   ros::ServiceClient m_client = n.serviceClient<global_map::SetMeridian>("SetMeridian");
   global_map::SetMeridian m_srv;

   uint16_t old_meridian = 0;

   while( fscanf(infile, "%lf,%lf", &lat, &lon) == 2 ) {
      ROS_INFO("Lat: %lf, Lon: %lf", lat, lon);
      uint16_t meridian = lon;

      if( meridian != old_meridian ) {
         ROS_INFO("New meridian: %d", meridian);
         old_meridian = meridian;
         m_srv.request.meridian = meridian;
         if( ! m_client.call(m_srv) ) {
            ROS_ERROR("Failed to call service SetMeridian");
         }
      }

      o_srv.request.lat = lat;
      o_srv.request.lon = lon;
      if( o_client.call(o_srv) ) {
         ROS_INFO("Row: %d, Col: %d", o_srv.response.row, o_srv.response.col);
      } else {
         ROS_ERROR("Failed to call service Offset");
      }
   }

   return 0;
}
