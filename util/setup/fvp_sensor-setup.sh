#!/bin/bash
#\file    fvp_sensor-setup.sh
#\brief   Generator of attachment files related to a FV+ sensor product.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.24, 2024

ros_bash_config="
#ROS Configuration
#ROS_DISTR=kinetic
ROS_DISTR=melodic
. /opt/ros/\$ROS_DISTR/setup.bash
. ~/catkin_ws/devel/setup.bash
. /opt/ros/\$ROS_DISTR/share/rosbash/rosbash

export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
#export ROS_MASTER_URI=http://10.10.6.203:11311
#export ROS_IP=10.10.6.203
export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:\$HOME/ros_ws:\${HOME}/prg/ay_test/ros
"

serial_code_set=
serial_code=
num_pairs=
answer_yes='n'
function print_usage
{
  echo "Usage: $0 [-y] SERIAL_CODE_SET NUM_PAIRS SERIAL_CODE_INDIVIDUAL"
  echo "  SERIAL_CODE_INDIVIDUAL: Serial code (4-digit number starting with 3) of the FV+ device to be configured with this execution."
  echo "  SERIAL_CODE_SET: Serial code that SERIAL_CODE_INDIVIDUAL belongs to."
  echo "  NUM_PAIRS: Number of pairs (sets) included in SERIAL_CODE_SET."
  exit 1
}
while getopts 'y' flag; do
  case "${flag}" in
    y) answer_yes='y' ;;
    *) print_usage ;;
  esac
done
shift $((OPTIND -1))
if [ $# -ne 3 ]; then
  print_usage
fi
serial_code_set=$1
num_pairs=$2
serial_code=$3

function ask
{
  while true; do
    echo -n '  (y|n) > '
    read s
    if [ "$s" == "y" ];then return 0; fi
    if [ "$s" == "n" ];then return 1; fi
  done
}

if ! [[ $serial_code =~ ^3[0-9]{3}$ ]]; then
  echo "Invalid code of the serial number (individual): $serial_code"
  exit 1
fi
if ! [[ $num_pairs =~ [0-9]+ ]]; then
  echo "Invalid number of pairs: $num_pairs"
  exit 1
fi
if ! [[ $serial_code_set =~ ^3[0-9]{3}$ ]]; then
  echo "Invalid code of the serial number (set): $serial_code_set"
  exit 1
fi
echo "Serial number (set) is: FVP$serial_code_set"
echo "Number of pairs in FVP$serial_code_set is: $num_pairs"
echo "Serial number (individual) is: FVP$serial_code"

set_name="FVP$serial_code_set(${num_pairs}set)"

echo "[$set_name]Generate launch and configuration files for FVP$serial_code?"
if ask; then
  eval "$ros_bash_config"
  roscd ay_fv_extra
  echo "In the package ay_fv_extra,"
  sed "s/fvp3XXX/fvp$serial_code/g" launch/fvp3XXX.launch > launch/fvp$serial_code.launch
  echo "  generated: launch/fvp$serial_code.launch"
  sed "s/fvp3XXX/fvp$serial_code/g" config/fvp3XXX.yaml > config/fvp$serial_code.yaml
  echo "  generated: config/fvp$serial_code.yaml"
fi

echo "[$set_name]Do you want to Launch the FV+ program for FVP$serial_code?"
echo "  Instruction: Plug in the FVP$serial_code device to this PC."
echo "  Note: DevID in config/fvp$serial_code.yaml should be matched."
echo "  Instruction: After the FV+ program is launched, do calibrations, and save an investigation video."
echo "  Instruction: After completed, quit with Ctrl+C."
if ask; then
  roslaunch ay_fv_extra fvp$serial_code.launch
  echo "Finished."
fi

echo "[$set_name]Do you want to copy files related to FVP$serial_code?"
if ask; then
  eval "$ros_bash_config"
  roscd ay_fv_extra
  dst_data=~/SensorKit/SensorKit_$set_name/Supplementary/ay_fv_extra
  dst_check=~/SensorKit/SensorKit_$set_name/Validation
  mkdir -p $dst_data/{config,launch,data_gen}
  mkdir -p $dst_check
  mv launch/fvp$serial_code.launch $dst_data/launch/
  mv config/fvp$serial_code.yaml $dst_data/config/
  if [ -f "data_gen/blob_fvp$serial_code.yaml" ];then
    mv data_gen/blob_fvp$serial_code.yaml $dst_data/data_gen/
  fi
  mv /tmp/vout-fvp$serial_code*.avi $dst_check/
  echo "Related data files are moved to: ~/SensorKit/SensorKit_$set_name/"
fi


