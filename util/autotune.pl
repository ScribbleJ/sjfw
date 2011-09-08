#!/usr/bin/perl
use strict;
use warnings;
use FileHandle;
use IPC::Open2;

our $| = 1;

my $hostpid = open2(\*HOSTREAD, \*HOSTWRITE, "/home/chris/reprap/sjfw/util/host.pl /dev/ttyACM0") || die("Cannot open host.");

my @pos = (0,0,0,0);

sub comandpos($) 
{
  my $com = shift;

  print HOSTWRITE $com . "\n";
  print HOSTWRITE "M114\n";
  
  while(my $line = <HOSTREAD>)
  {
    if($line =~ m/C: X:(\d+) Y:(\d+) Z:(\d+) A:(\d+)/)
    {
      $pos[0]=$1;
      $pos[1]=$2;
      $pos[2]=$3;
      $pos[3]=$4;
      print STDERR "us: " . $line;
      return;
    }
    else
    {
      print STDERR "Notus: " . $line;
    }
  }
}

sub printpos()
{
  print "X:$pos[0] Y:$pos[1] Z:$pos[2]\n";
}

printpos();
comandpos("G1 X10 F9000");
printpos();



