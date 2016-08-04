#!/bin/bash

# Usage: ./plotLog <file> <plotTitle> {gnuplotFile}

file=$1

paramfile=$3

if [ ! -f $paramfile ] || [ "$paramfile" == "" ]; then
    paramfile="plotParamTemplate"
fi

./curate.sh $file

newfile=$file.csv

cp $paramfile plotParams

sed -i -E 's,plotlog,'"$newfile"',g' plotParams

echo "set title \""$2"\"" >> plotParams

printf "\nreplot" >> plotParams
gnuplot plotParams > /dev/null
