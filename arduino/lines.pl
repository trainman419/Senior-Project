#!/usr/bin/perl

use strict;

my $comment = 0;
my $file = $ARGV;

my $type;
my $count = 0;
my $total = 0;

sub type() {
   $file =~ m/^.*?\.([^\.]+)$/;
   $type = $1;
}

while(<>) {
   if($file ne $ARGV) {
      if( length $file ) {
         print "$file: " if $file ne "-";
         print "$count countable lines\n";
      }
      $total += $count;

      $count = 0;
      $comment = 0;
      $file = $ARGV;
      type();
   }

   if( $type eq "c" or $type eq "cpp" or $type eq "h" ) {
      # count C/C++ lines
      
      # strip block comments
      s,/\*.*?\*/,,g;
      m,\*/, and $comment = 0;
      m,/\*, and $comment = 1;

      if(not $comment) {
         s,//.*,,;
         s,\s+,,g;
         s,^}$,,;
         s,^};$,,;
         s,^{$,,;
         $count++ if length;
      }
   } elsif( $type eq "S" ) {
      # count assembly lines
      s,;.*,,;
      s,\s+,,g;
      $count++ if length;
   }
}
print "$file: " if $file ne "-";
print "$count countable lines\n";

$total += $count;

print "$total total lines\n";
