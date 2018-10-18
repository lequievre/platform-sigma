#!/bin/bash

if [ $1 = "left" ]
then
	namespace="kuka_lwr_left"
else
	namespace="kuka_lwr_right"
fi

rostopic pub -1 /$namespace/kuka_group_command_controller_fri/command std_msgs/Float64MultiArray "data: [1.46,0.0,0.0,0.0,0.0,0.0,-2.96]"

read -n 1 -p "Press enter to send next position"

rostopic pub -1 /$namespace/kuka_group_command_controller_fri/command std_msgs/Float64MultiArray "data: [1.46,-2.0,0.0,0.0,0.0,0.0,-2.96]"

read -n 1 -p "Press enter to send next position"

rostopic pub -1 /lh/sh_lh_ffj0_position_controller/command std_msgs/Float64 --120
rostopic pub -1 /lh/sh_lh_mfj0_position_controller/command std_msgs/Float64 -- 95
rostopic pub -1 /lh/sh_lh_rfj0_position_controller/command std_msgs/Float64 -- 90
rostopic pub -1 /lh/sh_lh_lfj0_position_controller/command std_msgs/Float64 -- 70
rostopic pub -1 /lh/sh_lh_wrj1_position_controller/command std_msgs/Float64 -- 19
rostopic pub -1 /lh/sh_lh_wrj2_position_controller/command std_msgs/Float64 -- -2


