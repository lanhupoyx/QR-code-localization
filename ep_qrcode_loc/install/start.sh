#!/bin/bash


CRTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export LD_LIBRARY_PATH=$CRTDIR/lib:$LD_LIBRARY_PATH

#source /opt/libs-x86_64-Linux/libs_env.sh
#source /opt/ros/melodic/setup.bash
ROS_VERSION=$(rosversion -d)  # 获取ROS 1版本名称
SETUP_BASH="/opt/ros/$ROS_VERSION/setup.bash"
source $CRTDIR/../../setup.bash

cd $CRTDIR
nohup roslaunch --wait --timeout=60 ep_qrcode_loc run.launch > /dev/null 2>&1 &
