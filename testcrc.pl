#!/usr/bin/perl
my $string = $ARGV[0];
my $crc = 0;
foreach my $c (split('', $string))
{
  $crc ^= ord($c);
  printf("CRC +=%s %d\n", $c, $crc);
}

