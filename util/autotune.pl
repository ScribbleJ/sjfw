#!/usr/bin/perl
use strict;
use warnings;
use FileHandle;
use IPC::Run qw(run timeout start pump);

my $feed = 2000;
my $xmin = 1;
my $ymin = 1;






our $| = 1;

my ($HOSTWRITE, $HOSTREAD, $err);

my $h = start([qw(/home/chris/reprap/sjfw/util/host.pl /dev/ttyACM0)], \$HOSTWRITE, \$HOSTREAD, \$err, timeout(30)) or die("Can't start: $?");

my @pos = (0,0,0,0);

sub comandpos($) 
{
  my $com = shift;

  $HOSTWRITE .= "$com\n";
  $HOSTWRITE .= "M114\n";
  
  while(1)
  {
    pump $h;
    if($HOSTREAD =~ m/C: X:([\d\.-]+) Y:([\d\.-]+) Z:([\d\.-]+) A:([\d\.-]+)/)
    {
      $pos[0]=$1;
      $pos[1]=$2;
      $pos[2]=$3;
      $pos[3]=$4;
      print STDERR $HOSTREAD;
      $HOSTREAD = '';
      last;
    }
    else
    {
    }
    print STDERR $HOSTREAD;
    $HOSTREAD = '';
  }
}

sub printpos(@)
{
  my @pos = @_;
  print "X:$pos[0] Y:$pos[1] Z:$pos[2]\n";
}

my @sp = (0,0,0,0);
sub storepos() { @sp = @pos; }

comandpos("G1 X5 F$feed");
printpos(@pos);
comandpos("G1 X-10 F$feed");
printpos(@pos);



