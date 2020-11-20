#!/bin/bash

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout",  SYMLINK+="move_base"' >/etc/udev/rules.d/move_base_340.rules

service udev reload
sleep 1
service udev restart
