#!/usr/bin/perl
use strict;
use Device::SerialPort;

my $p = tie(*PH, 'Device::SerialPort', $ARGV[0]) || die("Cannot tie: $!\n");
$p->baudrate(115200);
$p->parity("none");
$p->databits(8);
$p->stopbits(1);
$p->handshake("none");
$p->write_settings || die("Can't write settings.\n");


# Resets the printer:
$p->pulse_dtr_on(100);

close PH || die("Close fail.\n");
untie *PH;
print("reset.\n");
