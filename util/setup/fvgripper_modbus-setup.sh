#!/bin/bash
#\file    fvgripper_modbus-setup.sh
#\brief   Script to setup a Linux PC for Modbus module for FV+GripperKit.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jul.27, 2023

answer_yes_admin='n'
answer_yes_user='n'
function print_usage
{
  echo "Usage: $0 [-y]"
  echo "  -y Answer yes to all queries about the system (admin) setup."
  echo "  -u Answer yes to all queries about the per-user setup."
}
while getopts 'uy' flag; do
  case "${flag}" in
    y) answer_yes_admin='y' ;;
    u) answer_yes_user='y' ;;
    *) print_usage
       exit 1 ;;
  esac
done

function ask
{
  while true; do
    echo -n '  (y|n) > '
    read s
    if [ "$s" == "y" ];then return 0; fi
    if [ "$s" == "n" ];then return 1; fi
  done
}
function ask_admin
{
  if [ "${answer_yes_admin}" == "y" ]; then return 0; fi
  if ask; then return 0; else return 1; fi
}
function ask_user
{
  if [ "${answer_yes_user}" == "y" ]; then return 0; fi
  if ask; then return 0; else return 1; fi
}

echo '[admin] Install Ubuntu packages?'
if ask_admin; then
  sudo apt -f install python-pip
fi

echo '[user] Install libraries in user space?'
if ask_user; then
  python -m pip install pymodbus
fi

echo '[user] Download the Modbus module for FV+GripperKit?'
fvgmodbus_dir=${HOME}/fvgripper_modbus
if ask_user; then
  echo -n '  server: '
  read server
  echo -n '  user: '
  read user
  scp -rp ${user}@${server}:'ros_ws/fvbiz/fvindustrial/fvgripper_modbus/fvgripper_modbus_srv.{py,sh}' ${fvgmodbus_dir}
fi

echo '[admin] Create the Modbus program links in /sbin?'
if ask_admin; then
  sudo ln -is ${fvgmodbus_dir}/fvgripper_modbus_srv.py /sbin/
  sudo ln -is ${fvgmodbus_dir}/fvgripper_modbus_srv.sh /sbin/
fi


# Configuration:
echo '
===================================

Instruction: Add the following line to sudoers.

%dialout ALL = NOPASSWD: /sbin/fix_usb_latency.sh, /sbin/iptables

'
echo '[admin] Run visudo?'
if ask; then
  sudo visudo
fi

echo '
===================================

Instruction: Add the following lines to ~/fv+config.sh

FVG_OPTS=--modbus

ROS_MASTER_URI=http://10.10.6.203:11311
ROS_IP=10.10.6.203
'
echo '[user] Edit ~/fv+config.sh?'
if ask; then
  nano ~/fv+config.sh
fi

