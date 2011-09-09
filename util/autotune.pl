#!/usr/bin/perl
# Simple script to automatically determine maximum feedrates and accel
# not complte -!
use strict;
use warnings;
use Data::Dumper qw{Dumper};
use FileHandle;
use IPC::Run qw(run timeout start pump);


our $| = 1;


my $feed = 2000;
my $max_feed = 14000;
my $initial_inc = 1000;
my $xmin = 1;
my $ymin = 1;
my $tests_in_run = 5;


my ($HOSTWRITE, $HOSTREAD, $err);

my $h = start([qw(/home/chris/reprap/sjfw/util/host.pl /dev/ttyACM0)], \$HOSTWRITE, \$HOSTREAD, \$err) or die("Can't start: $?");

my @pos = (0,0,0,0);

sub comm($) 
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
      #print STDERR $HOSTREAD;
      $HOSTREAD = '';
      last;
    }
    else
    {
    }
    #print STDERR $HOSTREAD;
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

sub dotestin($)
{
  my $f = shift;
  comm("G1 X50 F$feed");
  comm("G1 X-10 F$f");
  comm("G1 X-10 F$f");
  comm("G1 X-10 F$f");
  comm("G1 X-10 F$f");
  comm("G1 X-10 F$f");
  comm("G1 X-10 F$f");
  storepos();
  comm("G1 X-50 F$feed");
  comm("G92 X0");
}

sub dotestout($)
{
  my $f = shift;
  comm("G1 X10 F$f");
  comm("G1 X10 F$f");
  comm("G1 X10 F$f");
  comm("G1 X10 F$f");
  comm("G1 X10 F$f");
  comm("G1 X-60 F$feed");
  storepos();
  comm("G1 X-50 F$feed");
  comm("G92 X0");
}



sub dorun($$)
{
  my $f = shift;
  my $test = shift;

  comm("G91");
  comm("G1 X-100 F$feed");
  comm("G92 X0");

  my @samples = ();
  for(my $x=0;$x<$tests_in_run;$x++)
  {
    &$test($f);
    print "sample: " . $sp[0] . "\n";
    push @samples, $sp[0];
  }

  my $avg=0;
  foreach my $s (@samples)
  {
    $avg += $s;
  }
  $avg /= $tests_in_run;
  printf("Average X distance from stop: %4.4f\n", $avg);
  comm("M84");
  return $avg;
}


comm("M201 X$max_feed");
comm("M202 X$max_feed");

my @runs;
for(my $x=$feed;$x<$max_feed;$x+=$initial_inc)
{
  print("Starting run at feedrate: $x\n");
  push @runs, [$x, dorun($x,\&dotestin)];
}

foreach my $d (@runs)
{
  printf("Run: %d, %4.4f\n", $d->[0], $d->[1]);
}

print Dumper(@runs);






