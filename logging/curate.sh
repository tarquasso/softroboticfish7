#!/bin/bash

file=$1

newfile=${file}.csv

cp $file $newfile

sed -i -E "s/MISSION START.*$//g" $newfile
sed -i -E "s/cu:([0-9\.]+) ([0-9\.]+),/\1, \2,/g" $newfile
sed -i -E "s/[a-z_]+://g" $newfile
sed -i -E "s/  / /g" $newfile
