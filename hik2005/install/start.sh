#!/bin/bash


CRTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export LD_LIBRARY_PATH=$CRTDIR/lib:$LD_LIBRARY_PATH

#source /opt/libs-x86_64-Linux/libs_env.sh
source /opt/ros/melodic/setup.bash
source $CRTDIR/../../setup.bash

cd $CRTDIR
nohup roslaunch --wait --timeout=60 hik2005 run.launch > /dev/null 2>&1 &
