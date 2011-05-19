#!/usr/bin/perl

use strict;

my $x = 0;
my $y = 0;
my $t = 0;
my $c = 0;

my $foo;

my $pi = 3.14159265358;
my $pi_2 = $pi/2.0;

while(<>) {
   if( m/O\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)/ ) {
      $foo = $t;
#      $foo = -$c;
#      $foo = (-$c + $t)/2;
      $x += cos($foo)*$1 - sin($foo)*$2;
      $y += cos($foo)*$2 + sin($foo)*$1;
      $t -= $3*3.1;
      while( $t < 0 ) { $t += 2*$pi; }
      print "$x $y ";
      print ($t + $pi_2);
      print " $c\n";
   }
   if( m/C\s+(\-?\d+\.\d+)/ ) {
      $c = $1;
      if( $t == 0 ) { $t = -$c; }
      while($c > 0) { $c -= 2*$pi; }
   }
}
