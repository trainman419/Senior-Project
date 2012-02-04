#!/usr/bin/perl

use strict;

my $x = 0;
my $y = 0;
my $t = 0;
my $c = 0;

while(<>) {
   if( m/O\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)/ ) {
      $x += $1/10;
      $y += $2/10;
      $t += $3;
      print "$x $y $t $c\n";
   }
   if( m/C\s+(\-?\d+\.\d+)/ ) {
      $c = $1;
   }
}
