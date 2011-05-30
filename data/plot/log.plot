#!/usr/bin/gnuplot
l = strlen(in)
out = sprintf("%s.%s",substr(in, 0, l-4), type);

if( type eq "png" ) set terminal png size 1600,1200; else set terminal type
   
set output out;

set title "Odometry Data"
plot in using 1:2:(sin($4)):(cos($4)) with vectors title "Compass", \
   in using 1:2 with lines title "Path", \
   in using 1:2:(cos($3)):(sin($3)) with vectors title "Odometry"; 
