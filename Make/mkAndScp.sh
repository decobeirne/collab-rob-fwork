#!/bin/sh

echo "Cleaning and making gumstix target, copying to mmc/collabExp on gumstix"

if [ $# -ne 1 ]
then
echo "Found $# args, expected 1. Usage is: mkAndScp.sh <robotNumber>"
exit 1
fi

make cleanGumstix
make gumstix
scpStr="scp gumstix root@192.168.0."$1":/mmc/collabExp"
$scpStr

echo "Done"
