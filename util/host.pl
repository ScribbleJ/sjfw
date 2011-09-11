#!/usr/bin/perl
use strict;
use Device::SerialPort;
use IO::Select;

local $|=1;

my $use_sjfwcrc = 0;



my $SJFW_CRC = 0;

my $port = $ARGV[0] || die(usage());
my $baud = $ARGV[1] || 57600;


my $p = tie(*PH, 'Device::SerialPort', $ARGV[0]) || die("Cannot tie: $!\n");
$p->baudrate($baud);
$p->parity("none");
$p->databits(8);
$p->stopbits(1);
$p->handshake("none");
$p->dtr_active(0);
$p->write_settings || die("Can't write settings.\n");

# Reset not required for prusa - is require for tom.
# Resets the printer:
$p->pulse_dtr_on(100);
$p->pulse_dtr_off(100);

my $s = IO::Select->new(\*STDIN);

my $bufmax = 1;
my $bufsize = 0;

my $resend = 0;
my @linehist = ();
my $linenum = 0;
my $started = $ARGV[2] || 0;

my $instr='';
my $inready =0;

sub addcrc($$)
{
  my $l = shift;
  my $n = shift;

  $l = "N$n $l";
  my $ck = 0;
  foreach my $c (split('', $l))
  {
    $ck ^= ord($c);
  }
  if($SJFW_CRC)
  {
    $ck += length($l) + 128;
  }
  $l .= "*".$ck."\n";
}



my $t1 = time();

my $line = '';
while(1)
{
  if($resend and $bufsize < $bufmax)
  {
    my $numhist = scalar @linehist;
    if($resend > $numhist)
    {
      print "RESEND COMPLETE.\n";
      $resend = 0;
    }
    else
    {
      my ($rn, $rl) = ($linehist[$resend-1][0], $linehist[$resend-1][1]);
      $rl = addcrc($rl, $rn);
      print "REPEAT: " . $rl;
      print PH $rl;
      $resend++;
      $bufsize++;
    }
  }
  elsif($started == 1)
  {
    $started = 2;
    if($use_sjfwcrc == 1)
    {
      my $line = "M118 P1"; 
      push @linehist, [$linenum, $line];
      $line = addcrc($line, $linenum);
      print PH $line;
      print '> ' . $line;
      $bufsize++;
      $linenum++;
      $SJFW_CRC = 1;
    }
  }
  elsif($bufsize < $bufmax and scalar $s->can_read(0) and $started > 1)
  {
    my $char;
    if(sysread(STDIN,$char,1) != 1)
    {
      my $t2 = time();
      my $mins = ($t2 - $t1) / 60;
      printf("All done - took %4.4f mins.\n", $mins);
      sleep(30); # temporary hack
      die("Finish.");
    }
    $line .= $char;
    if($char eq "\n")
    {
      chomp $line;
      $line =~ s/\(.*$//o;
      $line =~ s/\;.*$//o;

      push @linehist, [$linenum, $line];

      $line = addcrc($line, $linenum);

      print PH $line;
      print '> ' . $line;

      $line = '';
      $bufsize++;
      $linenum++;
    }
  }
    
  my ($cin, $cch) = $p->read(1);
  if($cin == 1 and ord($cch) > 1)
  {
    $instr.=$cch;
    $inready = 1 if(ord($cch) < 32);
  }

  if($inready)
  {
    my $line = $instr;
    chop $line;
    $instr = '';
    $inready = 0;
    print '< ' . $line . "\n";

    if($line =~ m/^ok/)
    {
      $bufsize--;
    }
    elsif($line =~ m/Discard/)
    {
      $bufsize--;
    }
    elsif($line =~ m/^rs (\d+)/)
    {
      my $badline = $1;
      print "Need to resend $badline.\n";
      while((scalar @linehist) and $linehist[0][0] < $badline)
      {
        shift @linehist;
      }
      $resend = 1;
      $bufsize--;
    }
    elsif($line =~ m/start/)
    {
      if(!$started)
      {
        sleep(5);
        $started=1;
      }
    }
  }
}

close *PH || die("Close fail.\n");
untie *PH;

sub usage() { print "$0 /dev/ttyUSB0\n"; }
