#!/usr/bin/gnuplot
l = strlen(in)
in2 = sprintf("%s.%s",substr(in, 0, l-4), "com");
out = sprintf("dexter.%s", type);

#if( type eq "png" ) set terminal png size 1600,1200; else set terminal type
if( type eq "png" ) set terminal png size 1028,681; else set terminal type
   
set output out;

set title "Odometry Data"
set xlabel "X (meters)"
set ylabel "Y (meters)"
set style fill transparent solid 0.2 noborder
#set xrange [0:1521] (-78)
#set yrange [0:1118] (-81)
# width: ~113.10m (nominally [-60:10] )
# height: ~71.43m (nominally [-50:30] )
set xrange [-81.6:31.6]
set yrange [-50.72:20.71]
set xtics 10
set ytics 10
plot in using 1:2:3 with circles title "Position", \
   in using 1:2:(cos($4)/5):(sin($4)/5) with vectors title "Heading", \
   in using 1:2 with lines title "Path", \
   in2 using 1:2 with lines title "Odometry";
#   "dexter-bg.png" binary filetype=png;
#plot   "dexter-bg.png" binary filetype=png with rgbalpha;
#   "dexter-bg.rgb" binary array=950x600 flipy format='%uchar%uchar%uchar' with rgbimage notitle;

