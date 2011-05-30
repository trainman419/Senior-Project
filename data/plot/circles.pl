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

while(<>) {
   if( m/P\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)\s+(\-?\d+\.\d+)/ ) {
      $x = $1;
      $y = $2;

      $xx = $3;
      $xy = $4;
      $yx = $5;
      $yy = $6;

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
