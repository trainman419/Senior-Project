#!/usr/bin/perl

use strict;

my $x = 0;
my $y = 0;

my $xx = 0;
my $xy = 0;
my $yx = 0;
my $yy = 0;

my $t = 0;
my $tt = 0;
my $c = 0;

my $r = 0;

my $pi = 3.141592653;
my $pi2 = $pi / 2;

my $start = 0;
my $x_start = 0;
my $y_start = 0;

while(<>) {
   if( m/P\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)/ ) {
      $x = $1 / 10;
      $y = $2 / 10;

      if( not $start ) {
         $start = 1;
         $x_start = $x;
         $y_start = $y;
      }
      $x -= $x_start;
      $y -= $y_start;

      $xx = $3 / 10;
      $xy = $4 / 10;
      $yx = $5 / 10;
      $yy = $6 / 10;

      $r = $xx + $xy + $yx + $yy  / 4;

      $t = $pi2 - $7;
      $tt = $8;
      
      print "$x $y $r $t\n";
   }
   if( m/O\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)/ ) {
      $x += $1;
      $y += $2;
      $t += $3;
#      print "$x $y $t $c\n";
   }
   if( m/C\s+(\-?\d+\.\d+)/ ) {
      $c = $1;
   }
}
