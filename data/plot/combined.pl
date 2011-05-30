#!/usr/bin/perl

use strict;

my $x = 0;
my $y = 0;
my $t = 0;
my $c = 0;

my $start = 0;

while(<>) {
   if( m/O\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)/ ) {
      $x += $1;
      $y += $2;
      $t += $3;
      if( $start ) {
         print "$x $y $t $c\n";
      }
   }
   if( m/C\s+(\-?\d+\.\d+)/ ) {
      $c = $1;
   }
   if( m/P\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)/ ) {
      # pull starting position out of first postion message
      if( ! $start ) {
         $start = 1;
         $x = $1;
         $y = $2;
         $t = $7;
      }
   }
}
