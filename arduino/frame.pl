#!/usr/bin/perl

use strict;

# current function
my $fun;

my %frame; # frame size per gcc's comments
my %push;  # number of pushes in function
my %depth; # stack depth

my %calls; # call tree?

# parse assembly for stack depth and frame size
while(<>) {
   if( m/\.type\s+(.*?),\s+\@function/ ) {
      $fun = $1;
      not defined $push{$fun} and $push{$fun} = 0;
      not defined $frame{$fun} and $frame{$fun} = 0;
   }
   if( m|/\* frame size = (\d+) \*/| ) {
      $frame{$fun} = $1;
   }
   if( m/^\s+call (\S+)/ ) {
      $calls{$fun}->{$1}++;
   }
   if( m/^\s+push/ ) {
      $push{$fun}++;
   }
#   .size battery_thread, .-battery_thread
   if( m/\.size\s+$fun\,\s+\.\-$fun/ ) {
      $fun = "(null)";
   }
}

# compute stack depth for leaf functions
for my $key (sort keys %frame) {
   if( not defined $calls{$key} ) {
      $depth{$key} = $frame{$key} + $push{$key};
   }
}

# recursively compute max depth
sub depth($) {
   my ($key) = @_;
   if( defined $depth{$key} ) { # cache past results
      return $depth{$key};
   } else {
      my $max = 0;
      for my $call (keys %{$calls{$key}}) {
         if( not defined $frame{$call} and not defined $push{$call} ) {
            print "Unknown function call to $call\n";
         }
         if( depth($call) > $max ) {
            $max = depth($call);
         }
      }
      $depth{$key} = $max + $frame{$key} + $push{$key};
      return $depth{$key};
   }
}

for my $key (sort keys %frame) {
   if( defined $calls{$key} ) {
      $depth{$key} = depth($key);
   }
}

for my $key (sort keys %frame) {
#   print "\n$key:  $frame{$key}\n";
#   print "Calls:\n";
#   for my $call (sort keys %{$calls{$key}} ) {
#      print "$call ".$calls{$fun}->{$call}."\n";
#   }
#   print "Stack depth: $depth{$key}\n";
   printf "%30s\t%d\n", $key, $depth{$key};
}
