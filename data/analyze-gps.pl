#!/usr/bin/perl
#
# Read a raw GPS log, parse the GPRMC messages for validity and lat/lon
#  compute average and standard deviation of each (no correlations yet)
#  and output a more easily parsable list of lat/lon coordinates

use strict;

my @lat;
my @lon;

my $count = 0;
my $cumLat = 0;
my $cumLon = 0;
my $l1;
my $l2;

open( DATA, ">gps.data") or die "Can't open gps.data: $!";

open( KML, ">gps.kml" ) or die "Can't open gps.kml: $!";

print KML "<?xml version=\"1.0\" encoding=\"UTF-8\"?>
<kml xmlns=\"http://www.opengis.net/kml/2.2\">
<Document>
";

sub convert($) {
   my ($angle) = @_;
   $angle =~ m/(\d+?)(\d\d\.\d+)/;
   my $deg = $1;
   my $min = $2;
   return $deg + ($min/60);
}

while(<>) {
   if( m/\$GPRMC,\d+,A,(\d+\.\d+),N,(\d+\.\d+),W/ ) {
      $l1 = convert($1);
      $l2 = convert($2);

      push @lat, $l1;
      push @lon, $l2;
      $count++;
      $cumLat += $l1;
      $cumLon += $l2;

      print KML "<Placemark><Point><coordinates>-$l2,$l1</coordinates></Point></Placemark>\n";
#      print "Lat $1, Lon $2\n";
      print DATA "$l1, $l2\n";
   }
}

print KML "</Document></kml>\n";

close KML;

close DATA;

# conpute averages
my $avgLat = $cumLat / $count;
my $avgLon = $cumLon / $count;

# comput standard deviation of latitude
my $stdLat = 0;
for $l1 (@lat) {
   $stdLat += ( ($l1 - $avgLat) * ($l1 - $avgLat) );
}
$stdLat = sqrt($stdLat / $count );

# compute standard deviation of longitude
my $stdLon = 0;
for $l2 (@lon) {
   $stdLon += ( ($l2 - $avgLon) * ($l2 - $avgLon) );
}

$stdLon = sqrt( $stdLon / $count );

print "$count data points\n";
print "Average Latitue $avgLat, standard deviation $stdLat\n";
print "Average Longitude $avgLon, standard deviation $stdLon\n";

# output KML for polygon around average
open KML, ">gps-avg.kml" or die "Can't open gps-avg.kml: $!";
print KML "<?xml version=\"1.0\" encoding=\"UTF-8\"?>
<kml xmlns=\"http://www.opengis.net/kml/2.2\">
<Document>
";

print KML "<Placemark><Polygon>\n";
print KML "<altitudeMode>relativeToGround</altitudeMode>\n";
print KML "<outerBoundaryIs>\n";
print KML "<LinearRing><coordinates>\n";

# corner coordinates of polygon roughly encompassing 1 standard deviation
print KML "-".($avgLon - $stdLon).",$avgLat,1\n";
print KML "-$avgLon,".($avgLat + $stdLat).",1\n";
print KML "-".($avgLon + $stdLon).",$avgLat,1\n";
print KML "-$avgLon,".($avgLat - $stdLat).",1\n";
print KML "-".($avgLon - $stdLon).",$avgLat,1\n";

print KML "</coordinates></LinearRing>\n";
print KML "</outerBoundaryIs>\n";
print KML "</Polygon></Placemark>\n";
print KML "</Document></kml>\n";
