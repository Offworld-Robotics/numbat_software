#!/usr/bin/perl
# Converts protobuf .proto files to ros .msg files
# Author: Harry J.E Day
# Date:   9/08/14 

use strict; 
use warnings;

open(INPUT, "<$ARGV[0]");
open(OUTPUT, ">$ARGV[1]");



foreach my $line (<INPUT>) {
    $line =~ s/^package \w+\;.*$//g;
    $line =~ s/^.*(\{|\}).*$/ /g;
    $line =~ s/float/float32/g;
    $line =~ s/double/float64/g;
    $line =~ s/repeated//g;
    $line =~ s/required//g;
    $line =~ s/;//g;
    $line =~ s/\=.*//g;
    print OUTPUT $line;
}






    
