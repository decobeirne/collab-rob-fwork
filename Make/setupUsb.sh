#!/bin/sh

echo "Setting up USB connection and SSHing into gumstix"

if [ $# -ne 1 ]
then
echo "Found $# args, expected 1. Usage is: setupUsb.sh <robotNumber>"
exit 1
fi

modprobe usbnet
ifconfig usb0 192.168.0.1

echo "Use scp <fileName> root@192.168.0."$1":/<path> to copy stuff to gumstix"

sshStr="ssh root@192.168.0."$1
$sshStr


