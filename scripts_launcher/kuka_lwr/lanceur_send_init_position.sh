#!/bin/bash

# Laurent LEQUIEVRE
# laurent.lequievre@uca.fr
# UMR 6602 - Institut Pascal


cd /home/ifma/git_project/platform-sigma
source devel/setup.bash
if [ $1 = "left" ]
	then
		namespace="kuka_lwr_left"
	else
		namespace="kuka_lwr_right"
	fi
rostopic pub -1 /$namespace/kuka_group_command_controller_fri/command std_msgs/Float64MultiArray "data: [0.0,0.0,0.0,0.0,0.0,0.0,0.0]"





