#!/bin/bash
#\file    FV+GK-Setup.sh
#\brief   FV+GK setup tool.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.25, 2024

cd ~/Downloads
wget 'http://akihikoy.net/p/FVIncLogo/chrome-remote-desktop_current_amd64.deb' -O chrome-remote-desktop_current_amd64.deb
sudo dpkg -i chrome-remote-desktop_current_amd64.deb
sudo apt -y -f install

cd ~/Downloads
wget https://raw.githubusercontent.com/akihikoy/ay_common/master/util/setup/fv+gripper-setup.sh -O fv+gripper-setup.sh

bash ~/Downloads/fv+gripper-setup.sh -yu


cd ~/Downloads
wget https://raw.githubusercontent.com/fingervision-biz/fvpub/main/util/setup/fvgripper_modbus-setup.sh -O fvgripper_modbus-setup.sh

bash ~/Downloads/fvgripper_modbus-setup.sh -yu


