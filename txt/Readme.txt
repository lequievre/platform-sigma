Laurent LEQUIEVRE
laurent.lequievre@uca.fr

Juan Antonio Corrales Ramon
Juan-Antonio.Corrales-Ramon@sigma-clermont.fr

Sébastien Lengagne
sebastien.lengagne@univ-bpclermont.fr

Youcef Mezouar
youcef.mezouar@sigma-clermont.fr

Institut Pascal
MACCS Team (http://ip.univ-bpclermont.fr/index.php/fr/maccs)
UMR6602 Clermont Ferrand

In Real Mode
============
In real mode, the controllers are launched in separate computers.

-> On arms computer
(FRI - Fast Research Interface need to be run in sudo)
sudo -s su
source devel/setup.bash
roslaunch double_lwr_robot double_arms.launch

-> For left arm only
(FRI - Fast Research Interface need to be run in sudo)
sudo -s su
source devel/setup.bash
roslaunch double_lwr_robot double_arms.launch use_right_arm:=false

-> On pantilt computer
source devel/setup.bash
roslaunch pan_tilt_real pan_tilt_real.launch

You can move/control the robots like in simulation mode, so let's have a look to the simulation part.


In Simulation/Gazebo Mode
=========================

How to launch the simulation of Sigma Platform (arms + pantilt) :
---------------------------------------------------------------
source devel/setup.bash
roslaunch platform_gazebo platform_gazebo.launch

If you don't need the pantilt -> roslaunch platform_gazebo platform_gazebo.launch use_pantilt:=false


How to move the pantilt in position :
-----------------------------------
rostopic pub -1 /pantilt/pan_tilt_position_controller/command std_msgs/Float64MultiArray "data: [1.5,-0.5,0.5,0.5]"


How to move the right or left kuka lwr arm :
------------------------------------------

The namespace 'kuka_lwr_right' is used to control the right arm.
The namespace 'kuka_lwr_left' id used to control the left arm.

-> Get a list of ros services (ros controllers) available for the right arm (so need to use the 'kuka_lwr_right' namespace) :
rosservice call /kuka_lwr_right/controller_manager/list_controllers

you can do the same for the left arm -> rosservice call /kuka_lwr_left/controller_manager/list_controllers

An example of available controllers (loaded but stopped by default): (of course you can write your own controller !)

controller: 
  - 
    name: joint_state_controller
    state: running
    type: joint_state_controller/JointStateController
    hardware_interface: hardware_interface::JointStateInterface
    resources: []
  - 
    name: kuka_group_command_controller_fri
    state: stopped
    type: kuka_lwr_controllers/GroupCommandControllerFRI
    hardware_interface: hardware_interface::PositionJointInterface
    resources: ['kuka_lwr_right_0_joint', 'kuka_lwr_right_1_joint', 'kuka_lwr_right_2_joint', 'kuka_lwr_right_3_joint', 'kuka_lwr_right_4_joint', 'kuka_lwr_right_5_joint', 'kuka_lwr_right_6_joint']
  - 
    name: kuka_one_task_inverse_kinematics
    state: stopped
    type: kuka_lwr_controllers/OneTaskInverseKinematics
    hardware_interface: hardware_interface::PositionJointInterface
    resources: ['kuka_lwr_right_0_joint', 'kuka_lwr_right_1_joint', 'kuka_lwr_right_2_joint', 'kuka_lwr_right_3_joint', 'kuka_lwr_right_4_joint', 'kuka_lwr_right_5_joint', 'kuka_lwr_right_6_joint']
  - 
    name: cartesian_velocity_control
    state: stopped
    type: kuka_lwr_controllers/CartesianVelocityControl
    hardware_interface: hardware_interface::PositionJointInterface
    resources: ['kuka_lwr_right_0_joint', 'kuka_lwr_right_1_joint', 'kuka_lwr_right_2_joint', 'kuka_lwr_right_3_joint', 'kuka_lwr_right_4_joint', 'kuka_lwr_right_5_joint', 'kuka_lwr_right_6_joint']

-> How to Start the position controller 'kuka_group_command_controller_fri' for the 'kuka_lwr_right' arm :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['kuka_group_command_controller_fri'], stop_controllers: [], strictness: 2}"

The same for the left arm -> rosservice call /kuka_lwr_left/controller_manager/switch_controller "{start_controllers: ['kuka_group_command_controller_fri'], stop_controllers: [], strictness: 2}"


-> How to Send positions with the controller 'kuka_group_command_controller_fri' for the 'kuka_lwr_right' arm :
rostopic pub -1 /kuka_lwr_right/kuka_group_command_controller_fri/command std_msgs/Float64MultiArray "data: [0.9,0.9,0.5,0.5,0.3,0.5,0.8]"

The same for the left arm -> rostopic pub -1 /kuka_lwr_left/kuka_group_command_controller_fri/command std_msgs/Float64MultiArray "data: [0.9,0.9,0.5,0.5,0.3,0.5,0.8]"

-> How to Stop the position controller 'kuka_group_command_controller_fri' for the 'kuka_lwr_right' arm :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['kuka_group_command_controller_fri'], strictness: 1}"

-> How to Start 'cartesian inverse kinematic' controller :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['kuka_one_task_inverse_kinematics'], stop_controllers: [], strictness: 2}"

-> How to Send cartesian position :
rostopic pub -1 /kuka_lwr_right/kuka_one_task_inverse_kinematics/command kuka_lwr_controllers/PoseRPY '{id: 1, position: {x: -0.4, y: 0.3, z: 0.9}}'

-> How to Stop 'cartesian inverse kinematic' controller :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['kuka_one_task_inverse_kinematics'], strictness: 1}"


-> How to start 'kuka_gravity_compensation_controller' controller :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['kuka_gravity_compensation_controller'], stop_controllers: [], strictness: 2}"

-> How to stop 'kuka_gravity_compensation_controller' controller :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['kuka_gravity_compensation_controller'], strictness: 2}"

-> How to start 'torque_based_position_controller' controller :
rosservice call /kuka_lwr_left/controller_manager/switch_controller "{start_controllers: ['torque_based_position_controller'], stop_controllers: [], strictness: 2}"

-> How to stop 'torque_based_position_controller' controller :
rosservice call /kuka_lwr_left/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['torque_based_position_controller'], strictness: 2}"


RVIZ only
=========

roslaunch platform_rviz platform_rviz.launch


Kamal only
==========
-> For left arm only
(FRI - Fast Research Interface need to be run in sudo)
sudo -s su
cd ~/git_project/platform-sigma
source devel/setup.bash
roslaunch double_lwr_robot double_arms.launch use_right_arm:=false


-> How to start 'torque_based_position_controller' controller :
rosservice call /kuka_lwr_left/controller_manager/switch_controller "{start_controllers: ['torque_based_position_controller'], stop_controllers: [], strictness: 2}"

-> How to stop 'torque_based_position_controller' controller :
rosservice call /kuka_lwr_left/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['torque_based_position_controller'], strictness: 2}"

-> How to Send positions with the controller 'torque_based_position_controller' for the 'kuka_lwr_left' arm :
rostopic pub -1 /kuka_lwr_left/torque_based_position_controller/command std_msgs/Float64MultiArray "data: [0.9,0.9,0.5,0.5,0.3,0.5,0.8]"
rostopic pub -1 /kuka_lwr_left/torque_based_position_controller/command std_msgs/Float64MultiArray "data: [0.0,0.0,0.0,0.0,0.0,0.0,0.5]"

-> Get list of active services
rosservice call /kuka_lwr_left/controller_manager/list_controllers

-> To work from pantilt computer (set ros uri in every terminal window)
-> Inside each terminal window :
export ROS_MASTER_URI=http://ifma-kuka-test:11311
cd ~/git_project/platform-sigma
source devel/setup.bash


-> How to start 'torque_based_position_controller_gazebo' controller :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['torque_based_position_controller_gazebo'], stop_controllers: [], strictness: 2}"

-> How to stop 'torque_based_position_controller_gazebo' controller :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['torque_based_position_controller_gazebo'], strictness: 2}"


-> How to start 'kuka_gravity_compensation_controller' controller :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['kuka_gravity_compensation_controller'], stop_controllers: [], strictness: 2}"

-> How to stop 'kuka_gravity_compensation_controller' controller :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['kuka_gravity_compensation_controller'], strictness: 2}"


-> set kp and kd
rostopic pub -1 /kuka_lwr_left/torque_based_position_controller/setKp std_msgs/Float64MultiArray "data: [100,200,100,100,50,50,50]"

rostopic pub -1 /kuka_lwr_left/torque_based_position_controller/setKd std_msgs/Float64MultiArray "data: [50,50,50,50,10,10,10]"

rostopic pub -1 /kuka_lwr_right/torque_based_position_controller_gazebo/setStiffnessDamping kuka_lwr_controllers/StiffnessDamping '{ stiffness: { data: [200.0,200.0,200.0,200.0,200.0,200.0,200.0] } , damping: { data: [0.7,0.7,0.7,0.7,0.7,0.7,0.7] } }'

rostopic pub -1 /kuka_lwr_right/kuka_gravity_compensation_controller/setStiffnessDamping kuka_lwr_controllers/StiffnessDamping '{ stiffness: { data: [200.0,200.0,200.0,200.0,200.0,200.0,200.0] } , damping: { data: [0.7,0.7,0.7,0.7,0.7,0.7,0.7] } }'

rostopic pub -1 /kuka_lwr_right/kuka_gravity_compensation_controller/command std_msgs/Float64MultiArray "data: [0.0,0.0,0.0,0.0,0.0,0.0,0.0]"


Cartesian computed torque controller
====================================

-> Launch ros controllers and gazebo
roslaunch platform_gazebo platform_gazebo.launch

-> Go to a specific position to avoid singularity :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['kuka_group_command_controller_fri'], stop_controllers: [], strictness: 2}"

rostopic pub -1 /kuka_lwr_right/kuka_group_command_controller_fri/command std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0]"

rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['kuka_group_command_controller_fri'], strictness: 2}"

-> You can do the same actions with a specific bash script (launch from the workspace root) :
./src/platform_gazebo/start_specific_position.sh right "[0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0]"

-> Start/Stop 'cartesian_computed_torque_controller' :
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['cartesian_computed_torque_controller'], stop_controllers: [], strictness: 2}"

rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['cartesian_computed_torque_controller'], strictness: 2}"

-> Set PID gains :
rostopic pub -1 /kuka_lwr_right/cartesian_computed_torque_controller/set_gains std_msgs/Float64MultiArray "data: [500,500,500,500,500,500,500,100,100,100,100,100,100,100]"

-> Publish Pose :
rostopic pub -1 /kuka_lwr_right/cartesian_computed_torque_controller/command kuka_lwr_controllers/PoseRPY '{id: 1, position: {x: -0.5, y: 0.0, z: 0.9}}'

rostopic pub -1 /kuka_lwr_right/cartesian_computed_torque_controller/command kuka_lwr_controllers/PoseRPY '{id: 0, position: {x: -0.5, y: 0.0, z: 0.9}, orientation: {roll: ?, pitch: ?, yaw: ?}}'

rostopic pub -1 /kuka_lwr_left/cartesian_computed_torque_controller/set_gains std_msgs/Float64MultiArray "data: [3000,1800,1800,1400,100,100,100,181,62,53,54,10,10,10]"







