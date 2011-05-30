#!/usr/bin/gnuplot
set terminal png
l = strlen(in)
name = substr(in, 0, l-4)
out = sprintf("%s.png", name)
set output out

set title "Battery Levels"
plot in using 2 with lines title "Main Battery", in using 3 with lines title "Motor Battery"
