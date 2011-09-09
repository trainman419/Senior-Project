#!/usr/bin/perl

use strict;

my $fun;
my %frame;
my %calls;
my %depth;
my %push;

while(<>) {
   if( m/\.type\s+(.*?),\s+\@function/ ) {
      $fun = $1;
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

for my $key (sort keys %frame) {
   if( not defined $calls{$key} ) {
      $depth{$key} = $frame{$key} + $push{$key};
   }
}

sub depth($) {
   my ($key) = @_;
   if( defined $depth{$key} ) {
      return $depth{$key};
   } else {
      my $max = 0;
      for my $call (keys %{$calls{$key}}) {
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
