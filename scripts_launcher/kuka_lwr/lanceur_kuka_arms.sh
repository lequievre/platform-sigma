#!/bin/bash

# Laurent LEQUIEVRE
# laurent.lequievre@uca.fr
# UMR 6602 - Institut Pascal

cd /home/ifma/git_project/kuka-lwr-ros-sigma
source devel/setup.bash
roslaunch single_lwr_robot right_arm.launch
