#!/bin/bash

cd /home/iac_user/backup/kvlibsdk
sudo make uninstall 

make
sudo make install 

cd /home/iac_user/backup/linuxcan
sudo make dkms_uninstall

make dkms
sudo make dkms_install