/* test_offset.cpp
 *
 * A program to test the offset call; checking that conversion from 
 * lat/lon to grid indices is sane and proper.
 *
 * Author: Austin Hendrix
 */

#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <vector>

#include "ros/ros.h"
#include "global_map/Offset.h"
#include "global_map/SetMeridian.h"

using namespace std;

inline int min(int a, int b) {
   if( a < b ) 
      return a;
   return b;
}

inline int max(int a, int b) {
   if( a > b )
      return a;
   return b;
}

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

   int16_t old_meridian = 0;

   /*
   vector<int> rows;
   vector<int> cols;
   */
   vector<double> rows;
   vector<double> cols;

   double row_avg = 0;
   double col_avg = 0;
   int row_max = INT_MIN;
   int row_min = INT_MAX;
   int col_max = INT_MIN;
   int col_min = INT_MAX;
   int count = 0;

   int row;
   int col;

   ROS_INFO("row_avg size: %ld", sizeof(row_avg));

   while( fscanf(infile, "%lf,%lf", &lat, &lon) == 2 ) {
      //ROS_INFO("Lat: %lf, Lon: %lf", lat, lon);
      int16_t meridian = round(lon);

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
         row = o_srv.response.row;
         col = o_srv.response.col;
         row_avg += row;
         col_avg += col;
         row_min = min(row_min, row);
         row_max = max(row_max, row);
         col_min = min(col_min, col);
         col_max = max(col_max, col);
         count++;

         rows.push_back(row);
         cols.push_back(col);
         //ROS_INFO("Row: %d, Col: %d", o_srv.response.row, o_srv.response.col);
      } else {
         ROS_ERROR("Failed to call service Offset");
      }

      /*
      rows.push_back(lat);
      cols.push_back(lon);
      row_avg += lat;
      col_avg += lon;
      count++;
      */
   }

   row_avg /= count;
   col_avg /= count;

   long int row_var = 0;
   long int col_var = 0;

   vector<double>::iterator itr;

   for( itr = rows.begin(); itr != rows.end(); itr++ ) {
      row_var += (*itr - row_avg) * (*itr - row_avg);
   }

   for( itr = cols.begin(); itr != cols.end(); itr++ ) {
      col_var += (*itr - col_avg) * (*itr - col_avg);
   }

   row_var /= count;
   col_var /= count;

   ROS_INFO("Row Average: %lf", row_avg);
   ROS_INFO("Col Average: %lf", col_avg);
   ROS_INFO("Row Std Dev: %lf", sqrt(row_var));
   ROS_INFO("Col Std Dev: %lf", sqrt(col_var));

   FILE * outdata = fopen("subsample.csv", "w");

   // N samples, covering N consecutive seconds
   for( int N = 2; N < count; N++ ) {
      ROS_INFO("Subsample Size: %d", N);
      // subsampling: sample N consecutive measurements and take an average
      vector<double> row_avgs;
      vector<double> col_avgs;
      double subsample_row_var = 0;
      double subsample_col_var = 0;

      double subsample_row_std_avg = 0;
      double subsample_col_std_avg = 0;

      for( int start = 0; start < count - N; start++ ) {
         // compute average for this sample
         double tmp_row_avg = 0;
         double tmp_col_avg = 0;
         for( int i=0; i<N; i++ ) {
            tmp_row_avg += rows[i+start];
            tmp_col_avg += cols[i+start];
         }
         tmp_row_avg /= N;
         tmp_col_avg /= N;

         // compute variance for this sample
         double tmp_row_var = 0;
         double tmp_col_var = 0;
         for( int i=0; i<N; i++ ) {
            tmp_row_var += (tmp_row_avg - rows[i+start]) * (tmp_row_avg - rows[i+start]);
            tmp_col_var += (tmp_col_avg - cols[i+start]) * (tmp_col_avg - cols[i+start]);
         }

         tmp_row_var /= N;
         tmp_col_var /= N;
         subsample_row_std_avg += sqrt(tmp_row_var);
         subsample_col_std_avg += sqrt(tmp_col_var);

         row_avgs.push_back(tmp_row_avg);
         col_avgs.push_back(tmp_col_avg);

         subsample_row_var += (tmp_row_avg - row_avg) * (tmp_row_avg - row_avg);
         subsample_col_var += (tmp_col_avg - col_avg) * (tmp_col_avg - col_avg);
      }
      subsample_row_var /= (count-N);
      subsample_col_var /= (count-N);

      subsample_row_std_avg /= (count-N);
      subsample_col_std_avg /= (count-N);

      ROS_INFO("Subsample Row Std Dev: %lf", sqrt(subsample_row_var));
      ROS_INFO("Subsample Col Std Dev: %lf", sqrt(subsample_col_var));
      ROS_INFO("Subsample Row Std Dev Avg: %lf", subsample_row_std_avg);
      ROS_INFO("Subsample Col Std Dev Avg: %lf", subsample_col_std_avg);
      fprintf(outdata, "%d, %lf, %lf, %lf, %lf\n", N, sqrt(subsample_row_var), 
            sqrt(subsample_col_var), subsample_row_std_avg,
            subsample_col_std_avg);
   }
   fclose(outdata);
   /*ROS_INFO("Row min: %d", row_min);
     ROS_INFO("Row max: %d", row_max);
     ROS_INFO("Col min: %d", col_min);
     ROS_INFO("Col max: %d", col_max);*/

   return 0;
}
