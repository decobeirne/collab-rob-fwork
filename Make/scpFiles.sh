#!/bin/sh

echo "Copying experiment results files from FilesSync on gumstix to new local folder..."

if [ $# -ne 1 ]
then
echo "Found $# args, expected 1. Usage is: scpFiles.sh <robotNumber>"
exit 1
fi

localDirs[0]="../Files"
localDirs[1]=$(date +%Y%m%d)"_gumstixFiles"
localDirs[2]="gum$1"
localDirs[3]=${localDirs[0]}"/"${localDirs[1]}"/"${localDirs[2]}

echo "Copying files to ${localDirs[3]}/ ..."

if [ ! -d ${localDirs[0]} ]
then
echo "Creating dir "${localDirs[0]}
mkdir $dir0Str
fi

if [ ! -d ${localDirs[0]}"/"${localDirs[1]} ]
then
echo "Creating dir "${localDirs[0]}"/"${localDirs[1]}
mkdir ${localDirs[0]}"/"${localDirs[1]}
fi

if [ ! -d ${localDirs[3]} ]
then
echo "Creating dir "${localDirs[3]}
mkdir ${localDirs[3]}
fi

remoteDirStr="root@192.168.0."$1":/mmc/collabExp/FilesSync/*"
echo "Copying from $remoteDirStr ..."

scpStr="scp -r "$remoteDirStr" "${localDirs[3]}
$scpStr

echo "Done"

